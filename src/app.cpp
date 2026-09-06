/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "app.hpp"

#include "AOG_TC_iop_data.hpp"
#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_parameter_group_number_request_protocol.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"
#include "isobus/utility/iop_file_interface.hpp"
#include "isobus/utility/system_timing.hpp"

#include "task_controller.hpp"
#include "tractor_facilities.hpp"

#include "AOG_TC.iop.h"

#include "logging_utils.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <span>
#include <sstream>
#include <thread>

using boost::asio::ip::udp;

static constexpr std::uint8_t PREFERRED_TC_ADDRESS = isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer;

static std::string format_hex_address(std::uint8_t address)
{
	std::ostringstream value;
	value << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(address);
	return value.str();
}

// Helper: populate TimeDateInterface::TimeAndDate from the system clock.
// Used as the callback for TimeDateInterface to provide wall-clock time
// for PGN 65254 (FEE6) broadcasts.
static bool get_system_time(isobus::TimeDateInterface::TimeAndDate &td)
{
	auto now = std::chrono::system_clock::now();
	auto time_t_now = std::chrono::system_clock::to_time_t(now);
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
	            now.time_since_epoch())
	            .count() %
	  1000;

	std::tm tm_utc{};
	std::tm tm_local{};
#if defined(_WIN32)
	gmtime_s(&tm_utc, &time_t_now);
	localtime_s(&tm_local, &time_t_now);
#else
	gmtime_r(&time_t_now, &tm_utc);
	localtime_r(&time_t_now, &tm_local);
#endif

	// PGN 65254 (FEE6) requires the main fields to be UTC; localHourOffset/localMinuteOffset
	// are what a receiver adds to UTC to reconstruct local time. Derive the real, DST-aware
	// offset using only standard functions (avoid non-portable timegm/_mkgmtime).
	const std::time_t localSeconds = std::mktime(&tm_local);
	const std::time_t utcAsLocalSeconds = std::mktime(&tm_utc);
	const long offsetSeconds = static_cast<long>(localSeconds - utcAsLocalSeconds);

	td.year = static_cast<std::uint16_t>(tm_utc.tm_year + 1900);
	td.month = static_cast<std::uint8_t>(tm_utc.tm_mon + 1);
	td.day = static_cast<std::uint8_t>(tm_utc.tm_mday);
	td.hours = static_cast<std::uint8_t>(tm_utc.tm_hour);
	td.minutes = static_cast<std::uint8_t>(tm_utc.tm_min);
	td.seconds = static_cast<std::uint8_t>(tm_utc.tm_sec);
	td.milliseconds = static_cast<std::uint16_t>((ms / 250) * 250); // J1939: 0.25s resolution
	td.quarterDays = static_cast<std::uint8_t>(tm_utc.tm_hour / 6);
	td.localHourOffset = static_cast<std::int8_t>(offsetSeconds / 3600);
	td.localMinuteOffset = static_cast<std::int8_t>((offsetSeconds % 3600) / 60);
	return true;
}

// Diagnostic callback: log any Request for Repetition Rate (PGN 0xCC00)
// that is properly addressed to one of our control functions.  We never
// comply (return false) – this is purely for bus analysis.
static bool log_repetition_rate_request(
  std::uint32_t requestedPGN,
  std::shared_ptr<isobus::ControlFunction> requestingCF,
  std::shared_ptr<isobus::ControlFunction> /*targetCF*/,
  std::uint32_t repetitionRate,
  void * /*parentPointer*/)
{
	std::uint8_t srcAddr = requestingCF ? requestingCF->get_address() : 0xFF;
	std::cout << "[" << get_timestamp() << "] [Diag] Repetition-rate request from SA "
	          << static_cast<int>(srcAddr)
	          << ": PGN " << requestedPGN
	          << " (0x" << std::hex << requestedPGN << std::dec
	          << "), rate=" << repetitionRate << " ms" << std::endl;
	return false; // We don't comply; just logging.
}

// Diagnostic callback: log broadcast PGN 0xCC00 messages that AgIsoStack
// warns about ("malformed or broadcast request for repetition rate").
// These bypass the per-ICF callback, so we catch them with a global
// PGN listener.
static void log_broadcast_repetition_rate(
  const isobus::CANMessage &message,
  void * /*parentPointer*/)
{
	const auto &data = message.get_data();
	if (data.size() < 8)
	{
		return;
	}

	auto sourceCF = message.get_source_control_function();
	std::uint8_t srcAddr = sourceCF ? sourceCF->get_address() : 0xFF;

	// PGN is a 3-byte little-endian value at bytes 0-2.
	std::uint32_t requestedPGN =
	  static_cast<std::uint32_t>(data[0]) |
	  (static_cast<std::uint32_t>(data[1]) << 8) |
	  (static_cast<std::uint32_t>(data[2]) << 16);
	std::uint16_t requestedRate =
	  static_cast<std::uint16_t>(data[3]) |
	  (static_cast<std::uint16_t>(data[4]) << 8);

	std::cout << "[" << get_timestamp() << "] [Diag] Broadcast repetition-rate request from SA "
	          << static_cast<int>(srcAddr)
	          << ": PGN " << requestedPGN
	          << " (0x" << std::hex << requestedPGN << std::dec
	          << "), rate=" << requestedRate << " ms"
	          << " (broadcast – AgIsoStack will warn and ignore)" << std::endl;
}

// Enumerate and log all Control Functions on the bus
static void enumerate_bus_control_functions(const std::string &context)
{
	std::cout << "\n";
	log("Bus CFs") << context << std::endl;
	log("Bus CFs") << "==================================================" << std::endl;
	log("Bus CFs") << "Control Functions on ISOBUS:" << std::endl;
	log("Bus CFs") << "--------------------------------------------------" << std::endl;

	// Get all control functions; offline CFs are filtered out below.
	auto allCFs = isobus::CANNetworkManager::CANNetwork.get_control_functions(false);

	std::uint32_t cfCount = 0;

	for (const auto &cf : allCFs)
	{
		if (!cf)
			continue;

		// Only show control functions with valid addresses (online)
		if (!cf->get_address_valid())
			continue;

		cfCount++;
		isobus::NAME name = cf->get_NAME();
		bool isInternal = (cf->get_type() == isobus::ControlFunction::Type::Internal);

		log("Bus CFs") << "  Address: " << std::setw(3) << std::right << static_cast<int>(cf->get_address())
		               << " | Mfg: " << std::left << std::setw(5) << name.get_manufacturer_code()
		               << " | Func: " << std::left << std::setw(3) << static_cast<int>(name.get_function_code())
		               << " | IG: " << static_cast<int>(name.get_industry_group())
		               << " | Identity: " << std::setw(6) << std::right << name.get_identity_number()
		               << " | ECU Inst: " << static_cast<int>(name.get_ecu_instance())
		               << " | Func Inst: " << static_cast<int>(name.get_function_instance())
		               << (isInternal ? " [INTERNAL]" : "")
		               << std::endl;
	}

	if (cfCount == 0)
	{
		log("Bus CFs") << "  (No control functions detected on bus)" << std::endl;
	}

	log("Bus CFs") << "==================================================" << std::endl;
	log("Bus CFs") << "Total CFs found: " << cfCount << std::endl;
	std::cout << "\n";
}

// Check for TC address conflicts and log warning if we couldn't claim preferred address
static bool check_tc_address_conflict(const std::shared_ptr<isobus::InternalControlFunction> &ourTC)
{
	static std::uint32_t lastWarnTime = 0;
	static bool conflictDetected = false;
	bool conflictActive = false;
	const bool havePreferredAddress = ourTC && ourTC->get_address_valid() && (ourTC->get_address() == PREFERRED_TC_ADDRESS);

	if (ourTC && !havePreferredAddress)
	{
		// We don't have the preferred address - check if another TC has it.
		auto allCFs = isobus::CANNetworkManager::CANNetwork.get_control_functions(false);
		for (const auto &cf : allCFs)
		{
			if (cf && cf->get_address_valid() && (cf->get_address() == PREFERRED_TC_ADDRESS) && (cf != ourTC))
			{
				const isobus::NAME otherName = cf->get_NAME();
				const std::uint8_t funcCode = otherName.get_function_code();
				if (funcCode == static_cast<std::uint8_t>(isobus::NAME::Function::TaskController))
				{
					conflictActive = true;

					// Periodic warning every 30 seconds. Conflict detection itself is not throttled.
					if (isobus::SystemTiming::time_expired_ms(lastWarnTime, 30000))
					{
						std::cout << "\n";
						log("WARN") << "==================================================" << std::endl;
						log("WARN") << "TC ADDRESS CONFLICT - Another TC at preferred address " << static_cast<int>(PREFERRED_TC_ADDRESS) << std::endl;
						log("WARN") << "Conflicting TC: Mfg=" << otherName.get_manufacturer_code()
						            << ", Func=" << static_cast<int>(funcCode)
						            << ", Identity=" << otherName.get_identity_number()
						            << ", ECU Inst=" << static_cast<int>(otherName.get_ecu_instance())
						            << ", Func Inst=" << static_cast<int>(otherName.get_function_instance()) << std::endl;
						log("WARN") << "Our TC using address: " << static_cast<int>(ourTC->get_address()) << std::endl;
						log("WARN") << "==================================================" << std::endl;
						std::cout << "\n";
						lastWarnTime = isobus::SystemTiming::get_timestamp_ms();
					}
					break;
				}
			}
		}
	}

	if (conflictDetected && !conflictActive)
	{
		if (havePreferredAddress)
		{
			log("TC Address") << "Successfully claimed preferred address " << static_cast<int>(PREFERRED_TC_ADDRESS) << std::endl;
		}
		else
		{
			log("TC Address") << "TC address conflict resolved" << std::endl;
		}
	}
	conflictDetected = conflictActive;

	return conflictActive;
}

Application::Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver) :
  canDriver(canDriver)
{
}

bool Application::initialize()
{
	bool initialized = false;
	settings->load();

	if (setup_can_hardware() && setup_control_functions())
	{
		setup_task_controller_server();
		setup_tecu_interfaces();

		log() << "Task controller server started." << std::endl;

		if (settings->is_vt_enabled())
		{
			setup_vt_client();
		}
		else
		{
			log("Info") << "VT UI disabled in settings, skipping VT client." << std::endl;
		}

		setup_udp_connections();
		initialized = true;
	}

	return initialized;
}

bool Application::setup_can_hardware()
{
	if (nullptr == canDriver)
	{
		log() << "Unable to find a CAN driver. Please make sure the selected driver is installed." << std::endl;
		return false;
	}
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
	{
		log() << "Failed to start CAN hardware interface." << std::endl;
		return false;
	}

	isobus::CANNetworkManager::CANNetwork.get_configuration().set_number_of_packets_per_cts_message(255);

	// Start CAN network and allow a brief update window to observe bus CFs
	auto busDiscoveryWindowStart = isobus::SystemTiming::get_timestamp_ms();
	while (isobus::SystemTiming::get_time_elapsed_ms(busDiscoveryWindowStart) < 100)
	{
		isobus::CANNetworkManager::CANNetwork.update();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return true;
}

bool Application::setup_control_functions()
{
	// Enumerate CFs on the bus BEFORE creating our own functions
	enumerate_bus_control_functions("Before creating internal control functions");

	isobus::NAME ourNAME(0);

	//! Make sure you change these for your device!!!!
	ourNAME.set_arbitrary_address_capable(true);
	ourNAME.set_industry_group(2);
	ourNAME.set_device_class(0);
	ourNAME.set_identity_number(20);
	ourNAME.set_ecu_instance(0);
	ourNAME.set_function_instance(0); // TC #1. If you want to change the TC number, change this.
	ourNAME.set_device_class_instance(0);
	ourNAME.set_manufacturer_code(1407);

	isobus::NAME tcNAME = ourNAME;
	tcNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));

	isobus::NAME tecuNAME = ourNAME;
	tecuNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TractorECU));
	tecuNAME.set_arbitrary_address_capable(false); // TECU address is fixed
	tecuNAME.set_ecu_instance(0);

	log("Init") << "Creating Task Controller control function..." << std::endl;
	tcCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tcNAME, 0, isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer); // The preferred address for a TC is defined in ISO 11783

	// Wait for TC address claim with bounded wait loop (no async to avoid blocking on destruction)
	// Also implements minimum 250ms delay per J1939-81 section 4.4.4.1
	// CAs with addresses in range 128-247 must wait 250ms after claiming before sending other messages
	static constexpr std::uint32_t MINIMUM_ADDRESS_CLAIM_DELAY_MS = 250;
	static constexpr std::uint32_t MAX_ADDRESS_CLAIM_WAIT_MS = 5000;
	auto tcClaimWaitStart = isobus::SystemTiming::get_timestamp_ms();
	bool tcAddressClaimed = false;

	while (isobus::SystemTiming::get_time_elapsed_ms(tcClaimWaitStart) < MAX_ADDRESS_CLAIM_WAIT_MS)
	{
		isobus::CANNetworkManager::CANNetwork.update();
		if (tcCF->get_address_valid())
		{
			tcAddressClaimed = true;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	if (!tcAddressClaimed)
	{
		log() << "Failed to claim address for TC server. The control function might be invalid." << std::endl;
		return false;
	}

	// Record when the address was actually claimed for the 250ms delay calculation
	auto tcAddressClaimedTime = isobus::SystemTiming::get_timestamp_ms();
	log("Init") << "TC claimed address " << static_cast<int>(tcCF->get_address()) << std::endl;

	// Ensure minimum 250ms delay after address claim per J1939-81
	auto tcClaimElapsedMs = isobus::SystemTiming::get_time_elapsed_ms(tcAddressClaimedTime);
	if (tcClaimElapsedMs < MINIMUM_ADDRESS_CLAIM_DELAY_MS)
	{
		auto remainingDelay = MINIMUM_ADDRESS_CLAIM_DELAY_MS - tcClaimElapsedMs;
		log("Init") << "Waiting " << remainingDelay << "ms after address claim (J1939-81 250ms rule)..." << std::endl;
		// Process CAN messages during the delay to prevent timeouts
		auto delayStart = isobus::SystemTiming::get_timestamp_ms();
		while (isobus::SystemTiming::get_time_elapsed_ms(delayStart) < remainingDelay)
		{
			isobus::CANNetworkManager::CANNetwork.update();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	// Create TECU control function
	// TODO: Should we wait between this and TC?
	// TODO: If there's already a TECU on the bus we should not create ours
	if (tcCF && settings->is_tecu_enabled())
	{ // Only create TECU if TC was created and ECU is enabled
		log("Init") << "Creating Tractor ECU control function..." << std::endl;
		tecuCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(tecuNAME, 0, isobus::preferred_addresses::IndustryGroup2::TractorECU);

		// Wait for TECU address claim with minimum 250ms delay per J1939-81 section 4.4.4.1
		log("Init") << "Tractor ECU control function created, waiting for address claim..." << std::endl;

		// Update the network manager to process TECU CF claiming
		int tecuClaimAttempts = 0;
		const int MAX_TECU_CLAIM_ATTEMPTS = 50; // 5 seconds max
		while (!tecuCF->get_address_valid() && tecuClaimAttempts < MAX_TECU_CLAIM_ATTEMPTS)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			isobus::CANNetworkManager::CANNetwork.update();
			tecuClaimAttempts++;
		}

		// TECU is non-arbitrary-address-capable: it must land on its preferred address or not at all
		if (tecuCF->get_address_valid() && tecuCF->get_address() == isobus::preferred_addresses::IndustryGroup2::TractorECU)
		{
			// Record when the address was actually claimed for the 250ms delay calculation
			auto tecuAddressClaimedTime = isobus::SystemTiming::get_timestamp_ms();
			log("Init") << "TECU claimed address " << static_cast<int>(tecuCF->get_address()) << std::endl;

			// Ensure minimum 250ms delay after address claim per J1939-81
			auto tecuClaimElapsedMs = isobus::SystemTiming::get_time_elapsed_ms(tecuAddressClaimedTime);
			if (tecuClaimElapsedMs < MINIMUM_ADDRESS_CLAIM_DELAY_MS)
			{
				auto remainingDelay = MINIMUM_ADDRESS_CLAIM_DELAY_MS - tecuClaimElapsedMs;
				log("Init") << "Waiting " << remainingDelay << "ms after TECU address claim (J1939-81 250ms rule)..." << std::endl;
				// Process CAN messages during the delay to prevent timeouts
				auto delayStart = isobus::SystemTiming::get_timestamp_ms();
				while (isobus::SystemTiming::get_time_elapsed_ms(delayStart) < remainingDelay)
				{
					isobus::CANNetworkManager::CANNetwork.update();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}
		else
		{
			tecuAddressClaimFailed = true;
			if (tecuCF->get_address_valid())
			{
				log("Warning") << "TECU claimed unexpected address " << static_cast<int>(tecuCF->get_address()) << " instead of " << static_cast<int>(isobus::preferred_addresses::IndustryGroup2::TractorECU) << "!" << std::endl;
			}
			else
			{
				log("Warning") << "TECU failed to claim address " << static_cast<int>(isobus::preferred_addresses::IndustryGroup2::TractorECU) << "! Another TECU may be on the bus." << std::endl;
			}
			log("Warning") << "TECU functionality will be disabled." << std::endl;
			tecuCF.reset(); // Release the failed control function
		}
	}

	// Enumerate CFs on the bus AFTER creating our internal functions
	enumerate_bus_control_functions("After creating internal control functions");

	return true;
}

void Application::setup_task_controller_server()
{
	// Map settings version to TaskControllerVersion enum
	isobus::TaskControllerServer::TaskControllerVersion tcVersionEnum;
	switch (settings->get_tc_version())
	{
		case 0:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::DraftInternationalStandard;
			break;
		case 1:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::FinalDraftInternationalStandardFirstEdition;
			break;
		case 2:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::FirstPublishedEdition;
			break;
		case 3:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::SecondEditionDraft;
			break;
		case 4:
		default:
			tcVersionEnum = isobus::TaskControllerServer::TaskControllerVersion::SecondPublishedEdition;
			break;
	}

	tcServer = std::make_shared<MyTCServer>(tcCF, tcVersionEnum);
	auto &languageInterface = tcServer->get_language_command_interface();
	languageInterface.set_language_code(settings->get_language_code());
	languageInterface.set_country_code(settings->get_country_code());
	tcServer->initialize();
	tcServer->set_task_totals_active(true); // TODO: make this dynamic based on status in AOG
	tcFunctionalities = std::make_unique<isobus::ControlFunctionFunctionalities>(tcCF);
	tcFunctionalities->set_functionality_is_supported(
	  isobus::ControlFunctionFunctionalities::Functionalities::TaskControllerBasicServer,
	  1,
	  true);
	tcFunctionalities->set_functionality_is_supported(
	  isobus::ControlFunctionFunctionalities::Functionalities::TaskControllerGeoServer,
	  1,
	  false);
	tcFunctionalities->set_task_controller_geo_server_option_state(
	  isobus::ControlFunctionFunctionalities::TaskControllerGeoServerOptions::PolygonBasedPrescriptionMapsAreSupported,
	  false);
	tcFunctionalities->set_functionality_is_supported(
	  isobus::ControlFunctionFunctionalities::Functionalities::TaskControllerSectionControlServer,
	  1,
	  true);
	tcFunctionalities->set_task_controller_section_control_server_option_state(1, 64);

	// Announce Task Controller Tramline (TRACK) Server — Functionality 27
	// This tells the implement we support tramline control.
	// The AgIsoStack library doesn't have this enum value yet, so cast it directly.
	tcFunctionalities->set_functionality_is_supported(
	  static_cast<isobus::ControlFunctionFunctionalities::Functionalities>(27),
	  1, // Version 1
	  true);

	log("Init") << "TC announced TC-BAS, TC-SC (1 boom / 64 sections), and TC-TRAM (TRACK) via PGN 64654" << std::endl;

	// Register repetition-rate diagnostic on the TC's PGN request protocol
	auto tcPgnReq = tcCF->get_pgn_request_protocol().lock();
	if (tcPgnReq)
	{
		tcPgnReq->register_request_for_repetition_rate_callback(
		  0xFFFF /* Any PGN */, &log_repetition_rate_request, nullptr);
	}
}

void Application::setup_tecu_interfaces()
{
	// Initialize speed and distance messages
	if (tecuCF && tecuCF->get_address_valid())
	{
		log("Init") << "Creating TECU Control Function Functionalities..." << std::endl;
		tecuFunctionalities = std::make_unique<isobus::ControlFunctionFunctionalities>(tecuCF);
		// Announce as Basic Tractor ECU Server Class 1 (ISO 11783-9 compliance)
		tecuFunctionalities->set_functionality_is_supported(
		  isobus::ControlFunctionFunctionalities::Functionalities::BasicTractorECUServer,
		  2, // Generation 2 (Version 2)
		  true);
		tecuFunctionalities->set_basic_tractor_ECU_server_option_state(
		  isobus::ControlFunctionFunctionalities::BasicTractorECUOptions::Class1NoOptions,
		  true);
		log("Init") << "TECU announced as Class 1 Tractor ECU (PGN 64654)" << std::endl;

		log("Init") << "Creating Speed Messages Interface on TECU..." << std::endl;
		speedMessagesInterface = std::make_unique<isobus::SpeedMessagesInterface>(tecuCF, true, true, true, false); //TODO: make configurable whether to send these messages
		speedMessagesInterface->initialize();
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_implement_start_stop_operations_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::ImplementStartStopOperations::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_key_switch_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::KeySwitchState::NotAvailable);
		speedMessagesInterface->wheelBasedSpeedTransmitData.set_operator_direction_reversed_state(isobus::SpeedMessagesInterface::WheelBasedMachineSpeedData::OperatorDirectionReversed::NotAvailable);
		speedMessagesInterface->machineSelectedSpeedTransmitData.set_speed_source(isobus::SpeedMessagesInterface::MachineSelectedSpeedData::SpeedSource::NavigationBasedSpeed);
		log("Init") << "Speed Messages Interface created and initialized." << std::endl;

		log("Init") << "Creating NMEA2000 Message Interface on TECU..." << std::endl;
		nmea2000MessageInterface = std::make_unique<isobus::NMEA2000MessageInterface>(tecuCF, settings->is_nmea_send_enabled(), false, false, false, false, false, false);
		nmea2000MessageInterface->initialize();
		log("Init") << "NMEA2000 Message Interface created and initialized." << std::endl;

		// Initialize Tractor Facilities (PGN 65033 / 65032)
		tractorFacilities = std::make_unique<TractorFacilities>(tecuCF, settings);
		tractorFacilities->set_speed_messages_interface(speedMessagesInterface.get());
		tractorFacilities->set_nmea2000_message_interface(nmea2000MessageInterface.get());
		tractorFacilities->initialize();

		// Initialize TimeDateInterface for PGN 65254 (FEE6) broadcasting.
		// We broadcast FEE6 proactively so implements can discover us as a
		// time source without needing to send a REQRR. If another ECU is
		// already providing FEE6, we stay silent (duplicate provider detection).
		timeDateInterface = std::make_unique<isobus::TimeDateInterface>(tecuCF, get_system_time);
		timeDateInterface->initialize();

		// Listen for FEE6 from other ECUs to detect duplicate providers.
		// If we see FEE6 from another ECU, we suppress our own broadcast.
		timeDateInterface->get_event_dispatcher().add_listener(
		  [this](const isobus::TimeDateInterface::TimeAndDateInformation &info) {
			  if (info.controlFunction && tecuCF &&
			      info.controlFunction->get_address() != tecuCF->get_address())
			  {
				  // Fires on the isobus stack's background thread — see fee6Mutex's comment.
				  std::lock_guard<std::mutex> lock(fee6Mutex);
				  if (lastExternalFee6Ms == 0)
				  {
					  log("TECU") << "FEE6 provider detected at SA "
					              << static_cast<int>(info.controlFunction->get_address())
					              << " — suppressing our FEE6 broadcast" << std::endl;
					  if (fee6Broadcasting && tractorFacilities)
					  {
						  tractorFacilities->set_time_date_active(false);
						  fee6Broadcasting = false;
					  }
				  }
				  lastExternalFee6Ms = isobus::SystemTiming::get_timestamp_ms();
			  }
		  });
		log("Init") << "Time/Date interface (PGN 65254 / FEE6) created, interval="
		            << FEE6_TX_INTERVAL_MS << " ms" << std::endl;

		// Register repetition-rate diagnostic on the TECU's PGN request protocol
		auto tecuPgnReq = tecuCF->get_pgn_request_protocol().lock();
		if (tecuPgnReq)
		{
			tecuPgnReq->register_request_for_repetition_rate_callback(
			  0xFFFF /* Any PGN */, &log_repetition_rate_request, nullptr);
		}

		// Also catch broadcast PGN 0xCC00 messages that AgIsoStack warns about
		isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(
		  0xCC00 /* RequestForRepetitionRate */, &log_broadcast_repetition_rate, nullptr);
	}
	else
	{
		if (!settings->is_tecu_enabled())
		{
			log("Info") << "Tractor ECU disabled in settings, skipping ECU initialization." << std::endl;
		}
		else
		{
			log("Warning") << "TECU Control Function not available, Speed/NMEA interfaces not created" << std::endl;
		}
	}
}

void Application::setup_udp_connections()
{
	static std::uint8_t xteSid = 0;
	static std::uint32_t lastXteTransmit = 0;

	auto packetHandler = [this](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
		// PGN 0xD6 (214) — GPS/IMU data from AOG (src=0x7C, frame [0x80,0x81,0x7C,0xD6,...]).
		// Only the fix-quality byte is consumed today; the rest of the frame (position,
		// heading, speed, etc.) is not yet parsed by this TC.
		static constexpr std::size_t GNSS_FIX_QUALITY_OFFSET = 38; // frame byte 43
		static constexpr std::size_t MIN_0xD6_PAYLOAD_SIZE = GNSS_FIX_QUALITY_OFFSET + 1;
		if (src == 0x7C && pgn == 0xD6)
		{
			static std::uint8_t lastLoggedQuality = 0xFF;
			if (data.size() < MIN_0xD6_PAYLOAD_SIZE)
			{
				std::cout << "[" << get_timestamp() << "] [AOG] PGN 0xD6 received but too short for fix quality (len="
				          << data.size() << ")" << std::endl;
				return;
			}

			std::uint8_t quality = data[GNSS_FIX_QUALITY_OFFSET];
			// AOG fix values: 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK Fix, 5=Float, 6+=Estimated/Manual/Simulated.
			// 6+ are not valid ISOBUS quality levels but still represent an active (non-GPS) position
			// source — e.g. AOG's Simulator mode — so map them to the weakest real fix (1=GNSS)
			// rather than 0=No GPS, which would falsely claim there is no position at all.
			gnssFixQuality = (quality <= 5) ? quality : 1;
			lastGnssQualityMs = isobus::SystemTiming::get_timestamp_ms();

			if (quality != lastLoggedQuality)
			{
				std::cout << "[" << get_timestamp() << "] [TRACK][gnss] fix quality byte=" << static_cast<int>(quality)
				          << " -> DDI514=" << static_cast<int>(gnssFixQuality) << std::endl;
				lastLoggedQuality = quality;
			}
			return;
		}

		if (src != 0x7F)
		{
			return;
		}

		if (pgn == 0xE5 && data.size() >= 8) // 229 - 64 sections PGN
		{
			lastAogPacketMs = isobus::SystemTiming::get_timestamp_ms();
			std::vector<bool> sectionStates;
			for (std::uint8_t j = 0; j < 8; j++)
			{
				for (std::uint8_t i = 0; i < 8; i++)
				{
					sectionStates.push_back(data[j] & (1 << i));
				}
			}
			tcServer->update_section_states(sectionStates);
		}
		else if (pgn == 0xF1 && !data.empty()) // 241 - Section Control
		{
			lastAogPacketMs = isobus::SystemTiming::get_timestamp_ms();
			std::uint8_t sectionControlState = data[0];
			log() << "Received request from AOG to change section control state to " << (sectionControlState == 1 ? "enabled" : "disabled") << std::endl;
			tcServer->update_section_control_enabled(sectionControlState == 1);
			tcServer->update_track_control_enabled(sectionControlState == 1);
		}
		else if (pgn == 0xEF) // 239 - Machine Data
		{
			// Wire layout (payload bytes, for future reference — none of this is parsed today):
			//   [0]=uturn speed  [1]=hydLift  [2]=geoStop  [3]=TRAM (bit0=left marker, bit1=right marker)
			//   [4..]=section states, SC1-8 then SC9-16 (condensed bitfields)
			// Guidance/tramline state now comes exclusively from PGN 0xF4 (see GuidanceTrackProvider);
			// the tram bits here were only used by the removed synthetic fallback provider.
			lastAogPacketMs = isobus::SystemTiming::get_timestamp_ms();
		}
		else if (pgn == 0xF3) // 243 - Field Name
		{
			lastAogPacketMs = isobus::SystemTiming::get_timestamp_ms();

			// The whole payload IS the UTF-8 field name — no length prefix, no offset.
			// Confirmed against a live packet: a documented "length byte at offset 4,
			// name at offset 5+" layout does not match what AOG actually sends — the
			// payload was exactly N raw UTF-8 name bytes, nothing else. An empty
			// payload means the field is closed.
			if (data.empty())
			{
				if (hasActiveField)
				{
					log("Field") << "Field closed: " << currentFieldName << std::endl;
				}
				currentFieldName.clear();
				hasActiveField = false;
				// Invalidate any in-flight track context immediately — broadcasting
				// DDI 508 without a field to scope it to would defeat the point of the
				// field index folded into it below.
				currentTrackContext.valid = false;
			}
			else
			{
				constexpr std::size_t MAX_FIELD_NAME_BYTES = 248;
				std::size_t nameLength = data.size();
				if (nameLength > MAX_FIELD_NAME_BYTES)
				{
					log("Field") << "PGN 0xF3 name of " << nameLength << " bytes exceeds the documented "
					             << MAX_FIELD_NAME_BYTES << "-byte max; truncating." << std::endl;
					nameLength = MAX_FIELD_NAME_BYTES;
				}

				std::string fieldName(reinterpret_cast<const char *>(data.data()), nameLength);
				if (fieldName != currentFieldName || !hasActiveField)
				{
					currentFieldName = fieldName;
					currentFieldIndex = fieldRegistry.get_or_assign_index(fieldName);
					hasActiveField = true;
					log("Field") << "Field opened: " << currentFieldName << " (index " << currentFieldIndex << ")" << std::endl;
				}
			}
		}
		else if (pgn == 0xF4) // 244 - Guidance Track Context (AOG real guidance data)
		{
			lastAogPacketMs = isobus::SystemTiming::get_timestamp_ms();

			// Parse real guidance track context from AOG PGN 0xF4.
			// Always update currentTrackContext: when AOG sends valid=false
			// (guidance off / no active track), the context must be invalidated
			// so the TC stops broadcasting stale track data.
			// Note: parse() handles short-payload validation internally.
			currentTrackContext = trackProvider.parse(data);

			// AOG's own guidance reference ID (see GuidanceTrackProvider) is only unique
			// within whichever field AOG currently has open — fold in the field's own
			// persistent index (upper 16 bits) so DDI 508 is unique across fields too.
			// Without an active field, there's nothing to scope the ID to — don't send it.
			if (currentTrackContext.valid)
			{
				if (hasActiveField)
				{
					currentTrackContext.guidanceReferenceLineId =
					  (static_cast<std::uint32_t>(currentFieldIndex) << 16) |
					  (currentTrackContext.guidanceReferenceLineId & 0xFFFFu);
				}
				else
				{
					currentTrackContext.valid = false;
				}
			}
		}
		else if (pgn == 0xF2 && data.size() >= 6) // Process Data
		{
			lastAogPacketMs = isobus::SystemTiming::get_timestamp_ms();
			auto identifier = static_cast<isobus::DataDescriptionIndex>(data[0] | (data[1] << 8));

			const std::uint32_t rawValue =
			  static_cast<std::uint32_t>(data[2]) |
			  (static_cast<std::uint32_t>(data[3]) << 8) |
			  (static_cast<std::uint32_t>(data[4]) << 16) |
			  (static_cast<std::uint32_t>(data[5]) << 24);
			const std::int32_t value = static_cast<std::int32_t>(rawValue);
			if (identifier == isobus::DataDescriptionIndex::ActualSpeed)
			{
				lastSpeedValue = value; // Store the full precision value
				std::uint16_t speed = std::abs(value);
				auto direction = value < 0 ? isobus::SpeedMessagesInterface::MachineDirection::Reverse : isobus::SpeedMessagesInterface::MachineDirection::Forward;
				if (speedMessagesInterface)
				{
					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_direction_of_travel(direction);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_direction_of_travel(direction);

					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_speed(speed);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_speed(speed);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_speed(speed);
				}
				if (nmea2000MessageInterface)
				{
					auto &cog_sog_message = nmea2000MessageInterface->get_cog_sog_transmit_message();
					cog_sog_message.set_sequence_id(nmea2000SequenceIdentifier++);
					cog_sog_message.set_speed_over_ground(speed / 10);
					cog_sog_message.set_course_over_ground(0); // TODO: Implement course
					cog_sog_message.set_course_over_ground_reference(isobus::NMEA2000Messages::CourseOverGroundSpeedOverGroundRapidUpdate::CourseOverGroundReference::NotApplicableOrNull);
				}
			}
			else if (identifier == isobus::DataDescriptionIndex::GuidanceLineDeviation)
			{
				lastXteValue = value;
				std::int32_t xte = value / 1000; // Convert from mm to m
				static const std::uint8_t xteMode = 0b00000001;
				xteSid = xteSid % 253 + 1;

				std::uint8_t status = 0; // TODO: navigation terminated status

				std::array<std::uint8_t, 8> xteData = {
					xteSid, // Sequence ID
					static_cast<std::uint8_t>(xteMode | 0b00110000 | (status == 1 ? 0b00000000 : 0b01000000)), // XTE mode (4 bits) + Reserved (2 bits set to 1) + Navigation Terminated (2 bits)
					static_cast<std::uint8_t>(xte & 0xFF), // XTE LSB
					static_cast<std::uint8_t>((xte >> 8) & 0xFF), // XTE
					static_cast<std::uint8_t>((xte >> 16) & 0xFF), // XTE
					static_cast<std::uint8_t>((xte >> 24) & 0xFF), // XTE MSB
					0xFF, // Reserved byte 1 (all bits set to 1)
					0xFF // Reserved byte 2 (all bits set to 1)
				};
				if (isobus::SystemTiming::time_expired_ms(lastXteTransmit, 1000)) // Transmit every second
				{
					if (isobus::CANNetworkManager::CANNetwork.send_can_message(0x1F903, xteData.data(), xteData.size(), tcCF))
					{
						lastXteTransmit = isobus::SystemTiming::get_timestamp_ms();
					}
				}
			}
			else if (static_cast<std::uint16_t>(identifier) == 597 /*isobus::DataDescriptionIndex::TotalDistance*/)
			{
				lastDistanceMm = (value < 0) ? 0 : static_cast<std::uint32_t>(value);
				if (speedMessagesInterface)
				{
					speedMessagesInterface->groundBasedSpeedTransmitData.set_machine_distance(lastDistanceMm);
					speedMessagesInterface->wheelBasedSpeedTransmitData.set_machine_distance(lastDistanceMm);
					speedMessagesInterface->machineSelectedSpeedTransmitData.set_machine_distance(lastDistanceMm);
				}
			}
		}
	};
	udpConnections->set_packet_handler(packetHandler);
	udpConnections->open();

	log() << "UDP connections opened." << std::endl;
}

bool Application::update()
{
	static std::uint32_t lastHeartbeatTransmit = 0;

	udpConnections->handle_address_detection();
	udpConnections->handle_incoming_packets();

	tcServer->request_measurement_commands();
	tcServer->update();
	if (tcFunctionalities)
		tcFunctionalities->update();
	if (tecuFunctionalities)
		tecuFunctionalities->update();
	if (speedMessagesInterface)
		speedMessagesInterface->update();
	if (nmea2000MessageInterface)
		nmea2000MessageInterface->update();

	// Periodic FEE6 (Time/Date, PGN 65254) broadcast.
	// Only transmit if no other FEE6 provider is active on the bus.
	if (timeDateInterface && tecuCF && tecuCF->get_address_valid())
	{
		std::lock_guard<std::mutex> fee6Lock(fee6Mutex);
		const bool otherProviderActive =
		  (lastExternalFee6Ms != 0) &&
		  !isobus::SystemTiming::time_expired_ms(lastExternalFee6Ms, FEE6_PROVIDER_TIMEOUT_MS);

		if (otherProviderActive)
		{
			// Another ECU is broadcasting FEE6 — stay silent.
			if (fee6Broadcasting)
			{
				std::cout << "[" << get_timestamp() << "] [TECU] Stopping FEE6 broadcast; another provider active" << std::endl;
				fee6Broadcasting = false;
				if (tractorFacilities)
				{
					tractorFacilities->set_time_date_active(false);
				}
			}
		}
		else
		{
			// No other provider — broadcast FEE6 at our configured interval.
			if (!fee6Broadcasting)
			{
				std::cout << "[" << get_timestamp() << "] [TECU] Starting FEE6 broadcast (no other provider detected)" << std::endl;
				fee6Broadcasting = true;
				lastFee6TransmitMs = 0; // Force immediate first transmission
				if (tractorFacilities)
				{
					tractorFacilities->set_time_date_active(true);
				}
			}

			if (isobus::SystemTiming::time_expired_ms(lastFee6TransmitMs, FEE6_TX_INTERVAL_MS))
			{
				isobus::TimeDateInterface::TimeAndDate td;
				if (get_system_time(td))
				{
					if (timeDateInterface->send_time_and_date(td))
					{
						lastFee6TransmitMs = isobus::SystemTiming::get_timestamp_ms();
					}
				}
			}
		}
	}

	// Transmit PGN 65033 once on power-up (ISO 11783-7 B.24.3 repetition
	// rate: "on power-up, and then on request").
	if (!tractorFacilitiesSentOnPowerUp && tractorFacilities)
	{
		tractorFacilities->send_facilities_response();
		tractorFacilitiesSentOnPowerUp = true;
	}

	if (vtClient)
		update_vt_client();

	// Check for TC address conflicts every 15 seconds
	static std::uint32_t lastConflictCheck = 0;
	if (isobus::SystemTiming::time_expired_ms(lastConflictCheck, 15000))
	{
		const bool conflictActive = check_tc_address_conflict(tcCF);
		if (conflictActive)
		{
			send_hardware_message("TC address conflict: preferred address in use", 20, HW_MSG_ALERT);
		}
		else if (tcAddressConflictActive)
		{
			send_hardware_message("TC address conflict resolved", 5, HW_MSG_INFO);
		}
		tcAddressConflictActive = conflictActive;
		lastConflictCheck = isobus::SystemTiming::get_timestamp_ms();
	}

	// Diff active implement clients once per second so disconnect messages retain prior metadata.
	static std::uint32_t lastImplementScanMs = 0;
	if (isobus::SystemTiming::time_expired_ms(lastImplementScanMs, 1000))
	{
		std::map<std::uint64_t, ImplementSnapshot> currentImplementSnapshots;
		for (auto &client : tcServer->get_clients())
		{
			auto &state = client.second;
			if (state.get_number_of_sections() > 0)
			{
				const ImplementDetails details = derive_implement_details(state);
				currentImplementSnapshots[client.first->get_NAME().get_full_name()] = { details.displayName, details.sections, details.widthText };
			}
		}

		for (const auto &[name, implement] : currentImplementSnapshots)
		{
			if (implementSnapshots.find(name) == implementSnapshots.end())
			{
				std::ostringstream message;
				message << "Implement: " << implement.displayName << " (" << static_cast<int>(implement.sections) << " sections";
				if (!implement.widthText.empty())
				{
					message << ", " << implement.widthText << " m";
				}
				message << ")";
				send_hardware_message(message.str(), 5, HW_MSG_INFO);
			}
		}

		for (const auto &[name, implement] : implementSnapshots)
		{
			if (currentImplementSnapshots.find(name) == currentImplementSnapshots.end())
			{
				send_hardware_message("Implement lost: " + implement.displayName, 10, HW_MSG_ALERT);
			}
		}

		implementSnapshots = std::move(currentImplementSnapshots);
		lastImplementScanMs = isobus::SystemTiming::get_timestamp_ms();
	}

	// This is deferred from setup_control_functions() until UDP is ready.
	if (tecuAddressClaimFailed)
	{
		send_hardware_message("TECU failed to claim address " + std::to_string(static_cast<int>(isobus::preferred_addresses::IndustryGroup2::TractorECU)), 10, HW_MSG_ALERT);
		tecuAddressClaimFailed = false;
	}

	// Send section control heartbeat to AOG every 100ms (PGN 0xF0, source 0x80)
	// When no clients with sections, send 0 sections as heartbeat so AOG knows TC is alive
	if (isobus::SystemTiming::time_expired_ms(lastHeartbeatTransmit, 100))
	{
		bool anyClientWithSections = false;
		for (auto &client : tcServer->get_clients())
		{
			auto &state = client.second;

			// Skip clients with no sections (e.g., tractors or non-implement devices)
			if (state.get_number_of_sections() == 0)
			{
				continue;
			}

			anyClientWithSections = true;
			std::vector<uint8_t> data = { state.is_section_control_enabled(), state.get_number_of_sections() };

			std::uint8_t sectionIndex = 0;
			while (sectionIndex < state.get_number_of_sections())
			{
				std::uint8_t byte = 0;
				for (std::uint8_t i = 0; i < 8; i++)
				{
					if (sectionIndex < state.get_number_of_sections())
					{
						byte |= (state.get_section_actual_state(sectionIndex) == SectionState::ON) << i;
						sectionIndex++;
					}
				}
				data.push_back(byte);
			}
			udpConnections->send(0x80, 0xF0, data);
		}

		// If no clients with sections, send heartbeat with 0 sections
		if (settings->is_aog_heartbeat_enabled() && !anyClientWithSections)
		{
			std::vector<uint8_t> heartbeatData = { 0, 0 }; // section control disabled, 0 sections
			udpConnections->send(0x80, 0xF0, heartbeatData);
		}

		lastHeartbeatTransmit = isobus::SystemTiming::get_timestamp_ms();
	}

	// Send J1939 PGN 65256 every 100ms (0.1 seconds)
	if (isobus::SystemTiming::time_expired_ms(lastJ1939SpeedTransmit, 100) && tecuCF && tecuCF->get_address_valid())
	{
		std::uint16_t speed_j1939 = (static_cast<std::uint32_t>(std::abs(lastSpeedValue)) * 576u + 312u) / 625u; // mm/s -> (km/h)*256
		std::array<std::uint8_t, 8> j1939_speed_data = {
			0xFF,
			0xFF, // Compass Bearing (SPN 165)
			static_cast<std::uint8_t>(speed_j1939 & 0xFF),
			static_cast<std::uint8_t>((speed_j1939 >> 8) & 0xFF), // Machine Speed MSB (SPN 517)
			0xFF,
			0xFF, // Pitch (SPN 583)
			0xFF,
			0xFF // Altitude (SPN 580)
		};
		if (isobus::CANNetworkManager::CANNetwork.send_can_message(0xFEE8, j1939_speed_data.data(), j1939_speed_data.size(), tecuCF))
		{
			lastJ1939SpeedTransmit = isobus::SystemTiming::get_timestamp_ms();
		}
	}

	// Send Task Controller Status message every 2 seconds (ISO 11783-10 B.8.1)
	if (isobus::SystemTiming::time_expired_ms(lastTCStatusTransmit, 2000) && tcCF && tcCF->get_address_valid())
	{
		static bool firstStatusSent = false;
		send_task_controller_status_message();
		if (!firstStatusSent)
		{
			log("TC Status") << "First TC Status message sent (PGN 0xCB00)" << std::endl;
			firstStatusSent = true;
		}
	}

	// Send tramline track data to implement every 250 ms
	static std::uint32_t lastTramlineSendMs = 0;
	if (tcServer && isobus::SystemTiming::time_expired_ms(lastTramlineSendMs, 250))
	{
		// AOG only sends PGN 0xF4 when the guidance track actually changes (no heartbeat) —
		// long gaps between packets are the normal state while driving straight, not staleness.
		// Only clear the context on a real AOG disconnect (edge-triggered, not every tick).
		const bool aogConnectedNow = is_aog_connected();
		if (!aogConnectedNow && aogWasConnectedForTrack)
		{
			currentTrackContext.valid = false;
			trackProvider.reset(); // Treat next packet as fresh start after the gap
		}
		aogWasConnectedForTrack = aogConnectedNow;

		// GuidanceLineDeviation (DDI 0x0201) from AOG's XTE
		std::int32_t lineDevMm = lastXteValue;

		// GuidanceLineSwathWidth (DDI 0x0200) — 6000mm for ESPRO; TODO: derive from DDOP geometry
		std::int32_t swathMm = 6000;

		// PGN 0xD6 (GPS fix quality) is a separate, independently-timed stream from 0xF4 —
		// treat it as unknown if it goes stale on its own, even while AOG overall is connected.
		// Fall back to 1 (weakest real GNSS fix), not 0 (No GNSS): some implements gate
		// TRACK/section control on GNSS quality being non-zero, and AOG doesn't always
		// send 0xD6 at all (e.g. in Simulator mode) — 0 would falsely claim zero position
		// fix and can get commands rejected, exactly the failure the old hardcoded-4 hack
		// was working around.
		const bool gnssQualityFresh = (lastGnssQualityMs != 0) &&
		  !isobus::SystemTiming::time_expired_ms(lastGnssQualityMs, GNSS_QUALITY_TIMEOUT_MS);
		const std::uint8_t effectiveGnssQuality = gnssQualityFresh ? gnssFixQuality : 1;

		tcServer->send_tramline_track_data(currentTrackContext, swathMm, lineDevMm, effectiveGnssQuality);
		lastTramlineSendMs = isobus::SystemTiming::get_timestamp_ms();
	}

	return true;
}

bool Application::is_aog_connected() const
{
	return (lastAogPacketMs != 0) && !isobus::SystemTiming::time_expired_ms(lastAogPacketMs, AOG_CONNECTION_TIMEOUT_MS);
}

void Application::send_hardware_message(const std::string &text, std::uint8_t duration, std::uint8_t color)
{
	if (!text.empty())
	{
		static constexpr std::size_t MAX_TEXT_BYTES = 60;
		std::size_t textLength = text.size();
		if (textLength > MAX_TEXT_BYTES)
		{
			textLength = MAX_TEXT_BYTES;
			while ((textLength > 0) && ((static_cast<unsigned char>(text[textLength]) & 0xC0) == 0x80))
			{
				textLength--;
			}
		}

		const std::string message = text.substr(0, textLength);
		if (!message.empty())
		{
			std::vector<std::uint8_t> data = { duration, color };
			data.reserve(2 + message.size());
			data.insert(data.end(), message.begin(), message.end());
			const bool sent = udpConnections->send(0x80, 0xDD, data);

			log("Hardware Message") << (sent ? "" : "(send failed) ") << message << std::endl;
		}
	}
}

Application::ImplementDetails Application::derive_implement_details(ClientState &state) const
{
	ImplementDetails details;
	details.sections = state.get_number_of_sections();
	auto &pool = state.get_pool();
	if (auto deviceObject = pool.get_object_by_index(0))
	{
		const std::string designator = deviceObject->get_designator();
		if (!designator.empty())
		{
			details.displayName = designator;
		}
	}

	std::int32_t totalWidthMillimetres = 0;
	const auto geometry = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(pool);
	if (!geometry.booms.empty())
	{
		const auto &boom = geometry.booms.front();
		if (boom.xOffset_mm || boom.yOffset_mm)
		{
			std::ostringstream offsetText;
			offsetText << std::fixed << std::setprecision(2);
			if (boom.xOffset_mm)
			{
				offsetText << "X:" << std::showpos
				           << (static_cast<double>(boom.xOffset_mm.get()) / 1000.0)
				           << std::noshowpos;
			}
			else
			{
				offsetText << "X:n/a";
			}
			offsetText << " ";
			if (boom.yOffset_mm)
			{
				offsetText << "Y:" << std::showpos
				           << (static_cast<double>(boom.yOffset_mm.get()) / 1000.0)
				           << std::noshowpos;
			}
			else
			{
				offsetText << "Y:n/a";
			}
			details.boomOffsetText = offsetText.str();
		}
	}

	// section.width_mm/subBoom.width_mm only reflect a static Device Property (DPT) value
	// baked into the DDOP. Many implements report working width as a Device Process Data
	// (DPD) instead — which has no value in the DDOP at all, only a definition — so when the
	// static pool doesn't have it, fall back to whatever the client has actually reported
	// live over the bus (see ClientState::try_get_reported_working_width and the
	// ActualWorkingWidth/MaximumWorkingWidth/DefaultWorkingWidth cases in on_value_command).
	for (const auto &boom : geometry.booms)
	{
		for (const auto &section : boom.sections)
		{
			std::int32_t widthMm = 0;
			if (section.width_mm)
			{
				widthMm = section.width_mm.get();
			}
			else
			{
				state.try_get_reported_working_width(section.elementNumber, widthMm);
			}
			totalWidthMillimetres += widthMm;
		}
		for (const auto &subBoom : boom.subBooms)
		{
			if (subBoom.sections.empty())
			{
				std::int32_t widthMm = 0;
				if (subBoom.width_mm)
				{
					widthMm = subBoom.width_mm.get();
				}
				else
				{
					state.try_get_reported_working_width(subBoom.elementNumber, widthMm);
				}
				totalWidthMillimetres += widthMm;
			}
			for (const auto &section : subBoom.sections)
			{
				std::int32_t widthMm = 0;
				if (section.width_mm)
				{
					widthMm = section.width_mm.get();
				}
				else
				{
					state.try_get_reported_working_width(section.elementNumber, widthMm);
				}
				totalWidthMillimetres += widthMm;
			}
		}
	}
	if (totalWidthMillimetres > 0)
	{
		std::ostringstream widthText;
		widthText << std::fixed << std::setprecision(2) << (static_cast<double>(totalWidthMillimetres) / 1000.0);
		details.widthText = widthText.str();
	}

	return details;
}

void Application::send_task_controller_status_message()
{
	// ISO 11783-10 B.8.1 Task Controller Status message
	// PGN: 0xCB00 (Process Data), Command: 0x0E (Task Controller Status)
	// Transmission rate: 2 seconds, Global destination
	//
	// Byte 1: Bits 4-1 = 0x0E (Command), Bits 8-5 = 0x0F (Element nibble NA)
	// Byte 2: 0xFF (Element number MSB - not available)
	// Bytes 3-4: 0xFFFF (DDI - not available)
	// Byte 5: Status bits
	//   Bit 1 = Task totals active (1 = active, 0 = not active)
	//   Bit 2 = TC busy saving data
	//   Bit 3 = TC busy reading data
	//   Bit 4 = TC busy executing B.6 command
	//   Bit 8 = TC out of memory
	// Byte 6: Source address of client for B.6 command (0 if not applicable)
	// Byte 7: B.6 command being executed (0 if not applicable)
	// Byte 8: Reserved

	std::uint8_t statusByte = 0x00;
	if (tcServer && tcServer->get_task_totals_active())
	{
		statusByte |= 0x01; // Bit 1: Task totals active
	}
	// Bits 2-4 and 8 are always 0 for now (not busy, not out of memory)

	std::array<std::uint8_t, 8> tcStatusData = {
		0xFE, // Byte 1: Command 0x0E + Element nibble 0xF
		0xFF, // Byte 2: Element number MSB (not available)
		0xFF, // Byte 3: DDI LSB (not available)
		0xFF, // Byte 4: DDI MSB (not available)
		statusByte, // Byte 5: TC Status
		0x00, // Byte 6: Client SA for B.6 command (not applicable)
		0x00, // Byte 7: B.6 command being executed (not applicable)
		0xFF // Byte 8: Reserved
	};

	// Send to global destination (0xFF) - broadcast to all nodes
	// Using 4-arg version: PGN, data, length, source CF (destination is implicit in PGN for broadcast)
	const auto transmitAttemptTimestamp = isobus::SystemTiming::get_timestamp_ms();
	if (!isobus::CANNetworkManager::CANNetwork.send_can_message(0xCB00, tcStatusData.data(), tcStatusData.size(), tcCF))
	{
		log("TC Status") << "Failed to send TC Status message!" << std::endl;
	}

	// Update the transmit timestamp for every send attempt so failed sends
	// still respect the minimum 2-second transmit period.
	lastTCStatusTransmit = transmitAttemptTimestamp;
}

void Application::setup_vt_client()
{
	vtObjectPool.assign(std::begin(AOG_TC_IOP_DATA), std::end(AOG_TC_IOP_DATA));
	log("VT") << "Loaded embedded object pool (" << vtObjectPool.size() << " bytes)" << std::endl;

	// Partner filter for any Virtual Terminal server on the bus.
	const isobus::NAMEFilter filterVirtualTerminal(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	auto partnerVT = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, { filterVirtualTerminal });

	vtClient = std::make_shared<isobus::VirtualTerminalClient>(partnerVT, tcCF);

	// Include the scaling contract in the VT cache key so terminals do not
	// reuse an unscaled pool stored by an older build with the same IOP bytes.
	auto poolVersionData = vtObjectPool;
	const std::string scalingCacheSalt =
	  "scaled:" + std::to_string(ISO_MASK_SIZE) + ":" + std::to_string(ISO_DESIGNATOR_HEIGHT) + ":v1";
	poolVersionData.insert(poolVersionData.end(), scalingCacheSalt.begin(), scalingCacheSalt.end());
	const std::string poolHash = isobus::IOPFileInterface::hash_object_pool_to_version(poolVersionData);
	vtClient->set_object_pool(0, vtObjectPool.data(), static_cast<std::uint32_t>(vtObjectPool.size()), poolHash);
	vtClient->set_object_pool_scaling(0, ISO_MASK_SIZE, ISO_DESIGNATOR_HEIGHT);

	vtClient->get_vt_soft_key_event_dispatcher().add_listener([this](const isobus::VirtualTerminalClient::VTKeyEvent &event) {
		if (event.keyEvent != isobus::VirtualTerminalClient::KeyActivationCode::ButtonPressedOrLatched)
		{
			return;
		}

		std::uint16_t targetMask = 0xFFFF;
		switch (event.objectID)
		{
			case NavStatus:
				targetMask = DataMask_1000;
				break;
			case NavNetwork:
				targetMask = DataMask_Network;
				break;
			case NavImplement:
				targetMask = DataMask_Implement;
				break;
			case NavDiagnostics:
				targetMask = DataMask_Diagnostics;
				break;
			case NavConfig:
				targetMask = DataMask_Config;
				break;
			default:
				break;
		}

		if (targetMask != 0xFFFF)
		{
			vtClient->send_change_active_mask(WorkingSet_0, targetMask);
			log("VT") << "Navigating to mask " << targetMask << std::endl;
		}
	});
	vtClient->get_vt_button_event_dispatcher().add_listener([](const isobus::VirtualTerminalClient::VTKeyEvent &event) {
		log("VT") << "Button event, key=" << static_cast<int>(event.keyNumber) << std::endl;
	});
	vtUpdateHelper = std::make_unique<isobus::VirtualTerminalClientUpdateHelper>(vtClient);
	vtUpdateHelper->add_tracked_numeric_value(VTSpeedValue, 0);
	vtUpdateHelper->add_tracked_numeric_value(VTXteValue, 0);
	vtUpdateHelper->add_tracked_numeric_value(ConfigNmeaSend, settings->is_nmea_send_enabled());
	vtUpdateHelper->add_tracked_numeric_value(ConfigTecuEnabled, settings->is_tecu_enabled());

	vtClient->get_vt_change_numeric_value_event_dispatcher().add_listener([this](const isobus::VirtualTerminalClient::VTChangeNumericValueEvent &event) {
		const std::uint16_t objectID = event.objectID;
		const std::uint32_t value = event.value;
		const bool enabled = (value != 0);
		bool handled = true;
		bool saved = true;
		switch (objectID)
		{
			case ConfigNmeaSend:
				if (nmea2000MessageInterface)
				{
					saved = settings->set_nmea_send_enabled(enabled);
					if (saved)
					{
						nmea2000MessageInterface->set_enable_sending_cog_sog_cyclically(enabled);
					}
				}
				else
				{
					saved = false;
				}
				break;
			case ConfigTecuEnabled:
				saved = settings->set_tecu_enabled(enabled);
				break;
			default:
				handled = false;
				break;
		}
		if (handled)
		{
			log("VT") << (saved ? "Saved" : "Failed to save") << " configuration object " << objectID << " = " << enabled << std::endl;
		}
		if (handled && !saved)
		{
			bool storedValue = false;
			switch (objectID)
			{
				case ConfigNmeaSend:
					storedValue = settings->is_nmea_send_enabled();
					break;
				case ConfigTecuEnabled:
					storedValue = settings->is_tecu_enabled();
					break;
				default:
					break;
			}
			vtClient->send_change_numeric_value(objectID, storedValue ? 1U : 0U);
		}
	});
	vtUpdateHelper->initialize();

	log("VT") << "VT client created; waiting for a VT server on the bus." << std::endl;
}

void Application::send_vt_string_if_changed(std::uint16_t objectID, const std::string &value)
{
	auto cached = lastVtStrings.find(objectID);
	if (cached != lastVtStrings.end() && cached->second == value)
	{
		return;
	}
	vtClient->send_change_string_value(objectID, value);
	lastVtStrings[objectID] = value;
}

void Application::try_start_vt_client()
{
	auto vtPartner = vtClient->get_partner_control_function();
	if (vtPartner && vtPartner->get_address_valid())
	{
		vtClient->initialize(false);
		vtClientStarted = true;
		vtDisconnectedSinceMs = isobus::SystemTiming::get_timestamp_ms();
		log("VT") << "VT server detected at address " << static_cast<int>(vtPartner->get_address()) << ", VT client initialized (manual update mode)." << std::endl;
	}
}

void Application::handle_vt_disconnected()
{
	if (vtWasConnected)
	{
		vtDisconnectedSinceMs = isobus::SystemTiming::get_timestamp_ms();
		vtConnectionWarningLogged = false;
		vtCapabilitiesLogged = false;
	}
	else if (vtDisconnectedSinceMs == 0)
	{
		vtDisconnectedSinceMs = isobus::SystemTiming::get_timestamp_ms();
	}
	vtWasConnected = false;

	if ((!vtConnectionWarningLogged) &&
	    isobus::SystemTiming::time_expired_ms(vtDisconnectedSinceMs, 30000))
	{
		log("VT") << "WARNING: VT address was detected, but the client did not connect within 30 seconds. "
		          << "Reported VT capabilities: screen=" << vtClient->get_number_x_pixels() << "x" << vtClient->get_number_y_pixels()
		          << ", softkey=" << static_cast<int>(vtClient->get_softkey_x_axis_pixels()) << "x" << static_cast<int>(vtClient->get_softkey_y_axis_pixels())
		          << ", virtual softkeys=" << static_cast<int>(vtClient->get_number_virtual_softkeys())
		          << ", physical softkeys=" << static_cast<int>(vtClient->get_number_physical_softkeys())
		          << ". The pool requires five navigation softkeys. Check VT paging support and clear the terminal's cached/stored object pool before retrying."
		          << std::endl;
		vtConnectionWarningLogged = true;
	}

	lastVtStrings.clear();
	vtConfigSynced = false;
}

void Application::log_vt_capabilities_once()
{
	if (!vtCapabilitiesLogged)
	{
		const auto virtualSoftkeys = vtClient->get_number_virtual_softkeys();
		const auto physicalSoftkeys = vtClient->get_number_physical_softkeys();
		log("VT") << "Connected to VT version "
		          << static_cast<int>(vtClient->get_connected_vt_version())
		          << ": screen=" << vtClient->get_number_x_pixels() << "x" << vtClient->get_number_y_pixels()
		          << ", softkey=" << static_cast<int>(vtClient->get_softkey_x_axis_pixels()) << "x" << static_cast<int>(vtClient->get_softkey_y_axis_pixels())
		          << ", virtual softkeys=" << static_cast<int>(virtualSoftkeys)
		          << ", physical softkeys=" << static_cast<int>(physicalSoftkeys)
		          << std::endl;
		if (virtualSoftkeys < 5)
		{
			log("VT") << "WARNING: This object pool requires five virtual softkeys, but the VT reports "
			          << static_cast<int>(virtualSoftkeys) << "." << std::endl;
		}
		else if (physicalSoftkeys < 5)
		{
			log("VT") << "NOTICE: This object pool uses five navigation softkeys; this VT must provide softkey paging because it reports only "
			          << static_cast<int>(physicalSoftkeys) << " physical softkeys." << std::endl;
		}
		vtCapabilitiesLogged = true;
	}
}

void Application::sync_vt_config_once()
{
	if (!vtConfigSynced)
	{
		vtClient->send_enable_disable_object(
		  ConfigHydliftAuxN,
		  isobus::VirtualTerminalClient::EnableDisableObjectCommand::DisableObject);
		vtClient->send_enable_disable_object(
		  ConfigNmeaRead,
		  isobus::VirtualTerminalClient::EnableDisableObjectCommand::DisableObject);
		vtClient->send_enable_disable_object(
		  ConfigNmeaSend,
		  nmea2000MessageInterface ? isobus::VirtualTerminalClient::EnableDisableObjectCommand::EnableObject : isobus::VirtualTerminalClient::EnableDisableObjectCommand::DisableObject);
		vtClient->send_change_numeric_value(ConfigNmeaSend, settings->is_nmea_send_enabled());
		vtClient->send_change_numeric_value(ConfigTecuEnabled, settings->is_tecu_enabled());
		vtConfigSynced = true;
	}
}

void Application::update_vt_section_map()
{
	if (isobus::SystemTiming::time_expired_ms(lastVtSectionUpdateMs, 100))
	{
		std::string sectionMap = "No sections connected";
		auto clients = tcServer->get_clients(); // snapshot copy — see get_clients()'s declaration
		if (!clients.empty())
		{
			auto &state = clients.begin()->second;
			const auto sectionCount = std::min<std::uint8_t>(state.get_number_of_sections(), 64);
			if (sectionCount > 0)
			{
				sectionMap.clear();
				for (std::uint8_t section = 0; section < sectionCount; ++section)
				{
					if (section != 0)
					{
						sectionMap.push_back((section % 16) == 0 ? '\n' : ' ');
					}
					sectionMap.push_back(state.get_section_actual_state(section) == SectionState::ON ? '1' : '0');
				}
			}
		}
		send_vt_string_if_changed(ImplementSectionMap, sectionMap);
		lastVtSectionUpdateMs = isobus::SystemTiming::get_timestamp_ms();
	}
}

void Application::update_vt_client()
{
	if (!vtClientStarted)
	{
		try_start_vt_client();
		return;
	}

	vtClient->update();

	if (!vtClient->get_is_connected())
	{
		handle_vt_disconnected();
		return;
	}

	if (!vtWasConnected)
	{
		vtWasConnected = true;
		vtDisconnectedSinceMs = 0;
		vtConnectionWarningLogged = false;
	}

	log_vt_capabilities_once();

	if (!vtUpdateHelper)
	{
		return;
	}

	sync_vt_config_once();

	const bool aogConnected = is_aog_connected();
	vtUpdateHelper->set_numeric_value(VTSpeedValue, aogConnected ? static_cast<std::uint32_t>(std::abs(lastSpeedValue)) : 0U);

	vtUpdateHelper->set_numeric_value(VTXteValue, aogConnected ? (static_cast<std::uint32_t>(lastXteValue) ^ 0x80000000U) : 0x80000000U);
	vtUpdateHelper->set_numeric_value(ConfigNmeaSend, settings->is_nmea_send_enabled());
	vtUpdateHelper->set_numeric_value(ConfigTecuEnabled, settings->is_tecu_enabled());

	update_vt_section_map();

	if (!isobus::SystemTiming::time_expired_ms(lastVtStatusUpdateMs, 1000))
	{
		return;
	}
	update_vt_status_strings(aogConnected);
}

void Application::update_vt_status_strings(bool aogConnected)
{
	lastVtStatusUpdateMs = isobus::SystemTiming::get_timestamp_ms();

	send_vt_string_if_changed(VTWorkingSetStatusLabel, "AOG TC IP");
	send_vt_string_if_changed(VTAogIPStr, udpConnections->get_bound_ip_address());
	const std::string packetAge = (lastAogPacketMs == 0) ? "never" : (std::to_string(isobus::SystemTiming::get_time_elapsed_ms(lastAogPacketMs) / 1000) + " s");
	const bool taskRunning = tcServer->get_task_totals_active();
	auto clients = tcServer->get_clients(); // snapshot copy — see get_clients()'s declaration

	std::uint32_t totalSections = 0;
	for (const auto &client : clients)
	{
		totalSections += client.second.get_number_of_sections();
	}

	std::string implementName = "No implement";
	std::string sectionControl = "DISABLED";
	std::string workingWidth = "n/a";
	std::string boomOffset = "n/a";
	std::uint8_t implementSections = 0;
	int implementTramlineLevels = 0;

	// Find the first client with sections (the implement), skip tractors with 0 sections
	for (auto &client : clients)
	{
		if (client.second.get_number_of_sections() > 0)
		{
			auto &state = client.second;
			const ImplementDetails details = derive_implement_details(state);
			implementName = details.displayName;
			sectionControl = state.is_section_control_enabled() ? "ENABLED" : "DISABLED";
			implementSections = details.sections;
			workingWidth = details.widthText.empty() ? "n/a" : details.widthText;
			boomOffset = details.boomOffsetText.empty() ? "n/a" : details.boomOffsetText;
			implementTramlineLevels = state.get_supported_tramline_levels_bitmask();
			break; // Use the first implement with sections
		}
	}
	const std::string implementDisplayName = implementName.substr(0, 16);
	const std::string activeDDOP = clients.empty() ? "none" : implementDisplayName;
	if (boomOffset.size() > 16)
	{
		boomOffset.resize(16);
	}

	bool havePreferredAddress = tcCF && tcCF->get_address_valid() && (tcCF->get_address() == PREFERRED_TC_ADDRESS);
	std::string status = !havePreferredAddress ? "ADDR!" : (aogConnected ? "OK" : "OFF");
	send_vt_string_if_changed(VTWorkingSetStatusStr, status);

	// Count online control functions on the bus (ISOBUS network summary).
	const auto controlFunctions = isobus::CANNetworkManager::CANNetwork.get_control_functions(false);
	std::uint32_t cfCount = 0;
	for (const auto &cf : controlFunctions)
	{
		if (cf && cf->get_address_valid())
		{
			cfCount++;
		}
	}

	std::ostringstream mainImplementStatus;
	mainImplementStatus << "Name             " << implementDisplayName << '\n'
	                    << "Sections         " << totalSections << '\n'
	                    << "Section control  " << sectionControl;

	// Live guidance-track state from AOG + capability level from implement
	{
		std::string tramLive;
		if (!aogConnected)
		{
			tramLive = "n/a";
		}
		else if (currentTrackContext.valid)
		{
			std::ostringstream liveStr;
			liveStr << "ref:" << currentTrackContext.guidanceReferenceLineId
			        << " track:" << currentTrackContext.actualTrackNumber;
			tramLive = liveStr.str();
		}
		else
		{
			tramLive = "OFF";
		}

		std::string tramCaps;
		if (implementTramlineLevels & static_cast<int>(TramlineLevel::Level1))
			tramCaps += "L1 ";
		if (implementTramlineLevels & static_cast<int>(TramlineLevel::Level2))
			tramCaps += "L2 ";
		if (implementTramlineLevels & static_cast<int>(TramlineLevel::Level3))
			tramCaps += "L3 ";
		if (tramCaps.empty())
			tramCaps = "NONE";

		mainImplementStatus << "\nTram state       " << tramLive
		                    << "\nTram levels      " << tramCaps;
	}
	send_vt_string_if_changed(VTSectionsFromAOGS, mainImplementStatus.str());

	std::ostringstream distanceText;
	if (lastDistanceMm < 1000000U)
	{
		distanceText << std::fixed << std::setprecision(1) << (static_cast<double>(lastDistanceMm) / 1000.0) << " m";
	}
	else
	{
		distanceText << std::fixed << std::setprecision(2) << (static_cast<double>(lastDistanceMm) / 1000000.0) << " km";
	}

	std::ostringstream mainSystemStatus;
	mainSystemStatus << "Control funcs    " << cfCount << '\n'
	                 << "Task state       " << (taskRunning ? "RUNNING" : "STOPPED") << '\n'
	                 << "Direction        " << (!aogConnected ? "--" : (lastSpeedValue < 0 ? "REV" : "FWD")) << '\n'
	                 << "Distance         " << distanceText.str() << '\n'
	                 << "AOG packet age   " << packetAge;
	send_vt_string_if_changed(VTControlFunctionsStr, mainSystemStatus.str());
	send_vt_string_if_changed(ConfigHydliftLabel, "Hydlift: not impl.");
	send_vt_string_if_changed(ConfigNmeaReadLabel, "NMEA Read: not impl.");
	send_vt_string_if_changed(
	  ConfigNmeaSendLabel,
	  nmea2000MessageInterface ? (std::string("NMEA Send: ") + (settings->is_nmea_send_enabled() ? "ON" : "OFF")) : "NMEA Send: TECU req");
	send_vt_string_if_changed(
	  ConfigTecuLabel,
	  std::string("TECU: ") + (settings->is_tecu_enabled() ? "ON" : "OFF") + " (restart req)");

	std::ostringstream networkRows;
	std::uint8_t displayedCFs = 0;
	for (const auto &cf : controlFunctions)
	{
		if (displayedCFs >= 7)
		{
			break;
		}
		if (!cf || !cf->get_address_valid())
		{
			continue;
		}

		std::uint8_t sectionCount = 0;
		auto client = clients.find(cf);
		if (client != clients.end())
		{
			sectionCount = client->second.get_number_of_sections();
		}

		const auto name = cf->get_NAME();
		if (displayedCFs != 0)
		{
			networkRows << '\n';
		}
		networkRows << std::dec << std::nouppercase << std::setfill(' ') << std::left
		            << std::setw(6) << format_hex_address(cf->get_address())
		            << std::setw(10) << static_cast<int>(name.get_function_code())
		            << std::setw(6) << name.get_manufacturer_code()
		            << std::setw(9) << name.get_identity_number()
		            << static_cast<int>(sectionCount);
		displayedCFs++;
	}
	if (displayedCFs == 0)
	{
		networkRows << "No control functions online";
	}
	send_vt_string_if_changed(NetworkConnectedValue, std::to_string(cfCount));
	send_vt_string_if_changed(NetworkRows, networkRows.str());

	// Implement page: show the first active task-controller client and its section state.
	send_vt_string_if_changed(ImplementActiveDDOP, implementDisplayName);
	send_vt_string_if_changed(ImplementSectionControl, sectionControl);
	send_vt_string_if_changed(ImplementWorkingWidth, workingWidth);
	send_vt_string_if_changed(ImplementBoomOffset, boomOffset);
	send_vt_string_if_changed(ImplementSectionCount, std::to_string(implementSections));

	// Diagnostics page: compact state block plus one actionable alarm line.
	const std::string tcAddress = (tcCF && tcCF->get_address_valid()) ? format_hex_address(tcCF->get_address()) : "not claimed";
	send_vt_string_if_changed(DiagnosticsValues, aogConnected ? "ONLINE" : "OFFLINE");
	send_vt_string_if_changed(DiagnosticsLastPacket, packetAge);
	send_vt_string_if_changed(DiagnosticsTcAddress, tcAddress);
	send_vt_string_if_changed(DiagnosticsTaskState, taskRunning ? "RUNNING" : "STOPPED");
	send_vt_string_if_changed(DiagnosticsActiveDDOP, activeDDOP);
	send_vt_string_if_changed(DiagnosticsTcClients, std::to_string(clients.size()));

	std::string alarm = "No active alarms";
	if (!havePreferredAddress)
	{
		alarm = (tcCF && tcCF->get_address_valid()) ? "TC address conflict" : "TC address not claimed";
	}
	else if (!aogConnected)
	{
		alarm = "AOG heartbeat missing";
	}
	else if (clients.empty())
	{
		alarm = "Waiting for implement";
	}
	send_vt_string_if_changed(DiagnosticsAlarmValue, alarm);
}

void Application::stop()
{
	if (vtClient && vtClientStarted)
	{
		vtClient->terminate();
	}
	tcServer->terminate();
	isobus::CANHardwareInterface::stop();
}
