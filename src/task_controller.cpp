/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "task_controller.hpp"
#include "logging_utils.hpp"
#include "settings.hpp"

#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <bitset>
#include <fstream>
#include <iostream>
#include <set>

// Sanitize a string for use as a filename by replacing invalid characters with underscores
static std::string sanitize_filename(const std::string &input)
{
	std::string result = input;
	// Characters invalid on Windows (and / is invalid on POSIX too)
	const std::string invalid_chars = "\\/*:?\"<>|";
	for (char &c : result)
	{
		if (invalid_chars.find(c) != std::string::npos)
		{
			c = '_';
		}
	}
	return result;
}

void ClientState::set_number_of_sections(std::uint8_t number)
{
	numberOfSections = number;
	sectionSetpointStates.resize(number);
	sectionActualStates.resize(number);
	sectionToElementNumber.resize(number, 0); // Initialize all sections mapped to element 0 by default
}

void ClientState::set_section_setpoint_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionSetpointStates[section] = state;
	}
}

void ClientState::set_section_actual_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionActualStates[section] = state;
	}
}

std::uint16_t ClientState::get_element_number_for_section(std::uint8_t section) const
{
	if (section < numberOfSections && section < sectionToElementNumber.size())
	{
		return sectionToElementNumber[section];
	}
	return 0;
}

void ClientState::set_element_number_for_section(std::uint8_t section, std::uint16_t elementNumber)
{
	if (section < numberOfSections && section < sectionToElementNumber.size())
	{
		sectionToElementNumber[section] = elementNumber;
		elementToSection[elementNumber] = section;
	}
}

bool ClientState::try_get_section_for_element(std::uint16_t elementNumber, std::uint8_t &section) const
{
	auto it = elementToSection.find(elementNumber);
	if (it != elementToSection.end())
	{
		section = it->second;
		return true;
	}
	return false;
}

std::uint8_t ClientState::get_number_of_sections() const
{
	return numberOfSections;
}

std::uint8_t ClientState::get_section_setpoint_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		return sectionSetpointStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

std::uint8_t ClientState::get_section_actual_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		// For legacy per-element devices, sectionActualStates is updated directly
		// from DDI 141 in on_value_command, so we can return it without the
		// expensive parent-traversal check (which has thread-safety concerns
		// when called from the heartbeat on the main thread).
		if (usesPerElementControl)
		{
			return sectionActualStates[section];
		}

		// For modern/old devices using condensed DDIs, check parent hierarchy
		std::uint16_t elementNumber = get_element_number_for_section(section);
		if (is_element_or_parent_off(elementNumber))
		{
			return SectionState::OFF;
		}
		return sectionActualStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

bool ClientState::is_any_section_setpoint_on() const
{
	for (std::uint8_t state : sectionSetpointStates)
	{
		if (state == SectionState::ON)
		{
			return true;
		}
	}
	return false;
}

bool ClientState::get_setpoint_work_state() const
{
	return setpointWorkState;
}

void ClientState::set_setpoint_work_state(bool state)
{
	setpointWorkState = state;
}

bool ClientState::get_actual_work_state() const
{
	return actualWorkState;
}

void ClientState::set_actual_work_state(bool state)
{
	actualWorkState = state;
}

bool ClientState::is_section_control_enabled() const
{
	return isSectionControlEnabled;
}

void ClientState::set_section_control_enabled(bool state)
{
	isSectionControlEnabled = state;
}

bool ClientState::uses_per_element_control() const
{
	return usesPerElementControl;
}

void ClientState::set_uses_per_element_control(bool state)
{
	usesPerElementControl = state;
}

std::uint16_t ClientState::get_per_element_setpoint_ddi() const
{
	return perElementSetpointDDI;
}

void ClientState::set_per_element_setpoint_ddi(std::uint16_t ddi)
{
	perElementSetpointDDI = ddi;
}

isobus::DeviceDescriptorObjectPool &ClientState::get_pool()
{
	return pool;
}

bool ClientState::are_measurement_commands_sent() const
{
	return areMeasurementCommandsSent;
}

void ClientState::mark_measurement_commands_sent()
{
	areMeasurementCommandsSent = true;
}

std::uint16_t ClientState::get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	auto it = ddiToElementNumber.find(ddi);
	if (it != ddiToElementNumber.end())
	{
		return it->second;
	}
	std::cout << "[" << get_timestamp() << "] Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return 0;
}

void ClientState::set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
{
	ddiToElementNumber[ddi] = elementNumber;
}

bool ClientState::has_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	return ddiToElementNumber.find(ddi) != ddiToElementNumber.end();
}

bool ClientState::is_element_or_parent_off(std::uint16_t elementNumber) const
{
	std::set<std::uint16_t> visitedElements;
	auto &nonConstPool = const_cast<isobus::DeviceDescriptorObjectPool &>(pool);

	while (visitedElements.insert(elementNumber).second)
	{
		bool elementWorkState;
		if (try_get_element_work_state(elementNumber, elementWorkState) && !elementWorkState)
		{
			return true;
		}

		bool parentFound = false;
		std::uint16_t parentElementNumber = 0;
		for (std::uint32_t i = 0; (i < nonConstPool.size()) && !parentFound; i++)
		{
			auto object = nonConstPool.get_object_by_index(i);
			if (!object || (object->get_object_type() != isobus::task_controller_object::ObjectTypes::DeviceElement))
			{
				continue;
			}

			auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
			if (!elementObject)
			{
				continue;
			}

			for (std::uint16_t childId : elementObject->get_child_object_ids())
			{
				for (std::uint32_t j = 0; j < nonConstPool.size(); j++)
				{
					auto childObject = nonConstPool.get_object_by_index(j);
					if (!childObject || (childObject->get_object_id() != childId))
					{
						continue;
					}

					if (childObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
					{
						auto childElementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(childObject);
						if (childElementObject &&
						    (childElementObject->get_element_number() == elementNumber) &&
						    (elementObject->get_element_number() != elementNumber))
						{
							parentElementNumber = elementObject->get_element_number();
							parentFound = true;
						}
					}
					else if (childObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
					{
						auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(childObject);
						if (processDataObject)
						{
							auto ddi = static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi());
							if (has_element_number_for_ddi(ddi) &&
							    (get_element_number_for_ddi(ddi) == elementNumber) &&
							    (elementObject->get_element_number() != elementNumber))
							{
								parentElementNumber = elementObject->get_element_number();
								parentFound = true;
							}
						}
					}

					break;
				}
				if (parentFound)
				{
					break;
				}
			}
		}

		if (!parentFound)
		{
			return false;
		}
		elementNumber = parentElementNumber;
	}

	return false; // A cycle was found without an OFF element.
}

void ClientState::set_element_work_state(std::uint16_t elementNumber, bool isWorking)
{
	elementWorkStates[elementNumber] = isWorking;
}

bool ClientState::try_get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const
{
	auto it = elementWorkStates.find(elementNumber);
	if (it != elementWorkStates.end())
	{
		isWorking = it->second;
		return true;
	}
	return false;
}

MyTCServer::MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction,
                       isobus::TaskControllerServer::TaskControllerVersion version) :
  TaskControllerServer(internalControlFunction,
                       1, // AOG limits to 1 boom
                       64, // AOG limits to 16 sections of unique width but can be 64 by using zones
                       64, // 64 channels for position based control
                       isobus::TaskControllerOptions()
                         .with_implement_section_control(), // We support section control
                       version)
{
}

bool MyTCServer::activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &activationError, ObjectPoolErrorCodes &objectPoolError, std::uint16_t &parentObjectIDOfFaultyObject, std::uint16_t &faultyObjectID)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);

	// Default to "no error" / "no faulty object" up front, so every early-return path
	// below only needs to override what's actually wrong, per the meaning documented on
	// TaskControllerServer::activate_object_pool() in the base class.
	activationError = ObjectPoolActivationError::NoErrors;
	objectPoolError = ObjectPoolErrorCodes::NoErrors;
	parentObjectIDOfFaultyObject = isobus::NULL_OBJECT_ID;
	faultyObjectID = isobus::NULL_OBJECT_ID;

	log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name() << " requesting object pool activation" << std::endl;
	// Safety check to make sure partnerCF has uploaded a DDOP
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		// Not a DDOP content problem (there's no DDOP to have one) — this is a client
		// requesting activation without ever uploading, which isn't covered by a more
		// specific error code.
		activationError = ObjectPoolActivationError::AnyOtherError;
		return false;
	}

	// Initialize a new client state
	auto state = ClientState();
	// state.get_pool().set_task_controller_compatibility_level(get_active_client(partnerCF)->reportedVersion);
	state.get_pool().set_task_controller_compatibility_level(static_cast<std::uint8_t>(TaskControllerVersion::SecondEditionDraft));

	bool deserialized = false;
	while (!uploadedPools[partnerCF].empty())
	{
		auto binaryPool = uploadedPools[partnerCF].front();
		uploadedPools[partnerCF].pop();
		deserialized = state.get_pool().deserialize_binary_object_pool(binaryPool.data(), static_cast<std::uint32_t>(binaryPool.size()), partnerCF->get_NAME());
	}
	if (deserialized)
	{
		log() << "Successfully deserialized device descriptor object pool." << std::endl;

		// Save to NVM
		std::shared_ptr<isobus::task_controller_object::DeviceObject> deviceObject;
		for (std::uint16_t i = 0; i < state.get_pool().size(); i++)
		{
			auto object = state.get_pool().get_object_by_index(i);
			if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::Device)
			{
				deviceObject = std::static_pointer_cast<isobus::task_controller_object::DeviceObject>(object);
				break;
			}
		}

		if (!deviceObject)
		{
			// A spec-compliant pool always has exactly one Device object at its root.
			// If it's missing — a malformed pool, or a multi-chunk transfer that didn't
			// concatenate correctly upstream — reject the activation instead of crashing
			// on the dereference below. There's no single faulty object ID to point at
			// here (the problem is an absence, not a bad object), so parent/faulty stay
			// NULL_OBJECT_ID.
			log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name()
			                 << " activation REJECTED: deserialized pool (" << state.get_pool().size()
			                 << " objects) has no Device object." << std::endl;
			activationError = ObjectPoolActivationError::ThereAreErrorsInTheDDOP;
			objectPoolError = ObjectPoolErrorCodes::UnknownObjectReference;
			return false;
		}

		auto labelBytes = deviceObject->get_localization_label();
		std::string label(reinterpret_cast<const char *>(labelBytes.data()), labelBytes.size());
		// trim at first non-printable character (control chars, DEL, etc.)
		auto it = std::find_if(label.begin(), label.end(), [](unsigned char c) { return c < 0x20 || c >= 0x7F; });
		label.erase(it, label.end());

		auto fileName = std::to_string(partnerCF->get_NAME().get_full_name()) + "/" + sanitize_filename(label) + ".ddop";
		std::vector<std::uint8_t> binaryPool;
		if (state.get_pool().generate_binary_object_pool(binaryPool))
		{
			std::ofstream outFile(Settings::get_filename_path(fileName), std::ios::binary);
			if (outFile.is_open())
			{
				outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
				outFile.close();
				log() << "Saved DDOP to file: " << fileName << std::endl;
			}
			else
			{
				log() << "Unable to save DDOP to NVM. (Failed to open file) file: " << fileName << std::endl;
			}
		}
		else
		{
			log() << "Unable to save DDOP to NVM. (Failed to generate binary object pool)" << std::endl;
		}

		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(state.get_pool());
		std::uint8_t numberOfSections = 0;

		// Build a flat list of section element numbers in the same order as the geometry enumeration
		std::vector<std::uint16_t> sectionElementNumbers;

		std::cout << "Implement geometry: " << std::endl;
		std::cout << "Number of booms=" << implement.booms.size() << std::endl;
		for (const auto &boom : implement.booms)
		{
			std::cout << "Boom: id=" << static_cast<int>(boom.elementNumber) << std::endl;
			for (const auto &subBoom : boom.subBooms)
			{
				std::cout << "SubBoom: id=" << static_cast<int>(subBoom.elementNumber) << std::endl;
				for (const auto &section : subBoom.sections)
				{
					numberOfSections++;
					sectionElementNumbers.push_back(section.elementNumber);
					std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
					std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
					std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
					std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
					std::cout << "Width: " << section.width_mm.get() << std::endl;
				}
			}
			for (const auto &section : boom.sections)
			{
				numberOfSections++;
				sectionElementNumbers.push_back(section.elementNumber);
				std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
				std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
				std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
				std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
				std::cout << "Width: " << section.width_mm.get() << std::endl;
			}
		}
		state.set_number_of_sections(numberOfSections);

		// Map each section index to its element number from the geometry
		for (std::uint8_t i = 0; i < numberOfSections && i < sectionElementNumbers.size(); i++)
		{
			state.set_element_number_for_section(i, sectionElementNumbers[i]);
		}

		// Scan the DDOP to determine which section control method the device supports
		bool hasCondensedSetpoint = false; // Modern: DDI 290+ (paired with DDI 289 for global work state)
		bool hasSettableCondensedActual = false; // Old: DDI 161+ settable
		bool hasSettableActualWorkState = false; // Oldest: DDI 141 per-element settable

		for (std::uint32_t i = 0; i < state.get_pool().size(); i++)
		{
			auto object = state.get_pool().get_object_by_index(i);
			if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
			{
				auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
				auto ddi = processDataObject->get_ddi();

				if (ddi >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) &&
				    ddi <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256))
				{
					hasCondensedSetpoint = true;
				}
				if (ddi >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
				    ddi <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256) &&
				    processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
				{
					hasSettableCondensedActual = true;
				}
				if (ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) &&
				    processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
				{
					hasSettableActualWorkState = true;
				}
			}
		}

		// Announce and configure the section control method (hierarchy: Modern > Old > Oldest)
		if (hasCondensedSetpoint)
		{
			// Modern: condensed setpoint DDI 290+ (always paired with DDI 289 for global work state)
			log("TC Server") << "Attempting Section Control via: DDI 290 (SetpointCondensedWorkState) + DDI 289 (SetpointWorkState)"
			                 << " for " << static_cast<int>(numberOfSections) << " sections." << std::endl;
		}
		else if (hasSettableCondensedActual)
		{
			// Old: settable condensed actual DDI 161+
			log("TC Server") << "Attempting Section Control via: DDI 161 (ActualCondensedWorkState, settable)"
			                 << " for " << static_cast<int>(numberOfSections) << " sections." << std::endl;
		}
		else if (hasSettableActualWorkState)
		{
			// Oldest: per-element settable DDI 141
			state.set_uses_per_element_control(true);
			state.set_per_element_setpoint_ddi(static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState));
			log("TC Server") << "Attempting Section Control via: DDI 141 (ActualWorkState, settable per-element)"
			                 << " for " << static_cast<int>(numberOfSections) << " sections." << std::endl;
			for (std::uint8_t i = 0; i < numberOfSections; i++)
			{
				std::cout << "  Section " << static_cast<int>(i) << " -> element " << sectionElementNumbers[i] << std::endl;
			}
		}
		else
		{
			log("TC Server") << "WARNING: No supported section control method detected! "
			                 << "Device has no DDI 290, 161 (settable), or 141 (settable)." << std::endl;
		}
	}
	else
	{
		log() << "Failed to deserialize device descriptor object pool." << std::endl;
		activationError = ObjectPoolActivationError::ThereAreErrorsInTheDDOP;
		objectPoolError = ObjectPoolErrorCodes::AnyOtherError;
		return false;
	}

	clients[partnerCF] = state;
	log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name() << " registered successfully with "
	                 << static_cast<int>(state.get_number_of_sections()) << " sections." << std::endl;
	return true;
}

bool MyTCServer::change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &)
{
	return true;
}

bool MyTCServer::deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &)
{
	return false;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &)
{
	return false;
}

bool MyTCServer::get_is_enough_memory_available(std::uint32_t)
{
	return true;
}

void MyTCServer::identify_task_controller(std::uint8_t tcNumber)
{
	// ISO 11783-10 B.5.4/B.5.5: Identify Task Controller message
	// When this is called, the TC shall display its TC number for 3 seconds
	// TC Number = Function Instance + 1 (range 1-32)
	//
	// Since this is a console application without GUI, we log to console
	// In a GUI application, this would display the TC number visually
	auto timestamp = get_timestamp();
	std::cout << "[" << timestamp << "] ========================================" << std::endl;
	std::cout << "[" << timestamp << "] === TC NUMBER " << static_cast<int>(tcNumber) << " IDENTIFIED ===" << std::endl;
	std::cout << "[" << timestamp << "] ========================================" << std::endl;
}

void MyTCServer::on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	// Cleanup the client state
	std::cout << "[" << get_timestamp() << "] [TC Server] Client " << partner->get_NAME().get_full_name() << " has timed out!" << std::endl;
	clients.erase(partner);
}

void MyTCServer::on_client_version_received(std::shared_ptr<isobus::ControlFunction> clientControlFunction, std::uint8_t version)
{
	std::cout << "[" << get_timestamp() << "] [TC Server] Client " << clientControlFunction->get_NAME().get_full_name()
	          << " reported TC version " << static_cast<int>(version) << std::endl;
}

void MyTCServer::on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner,
                                             std::uint16_t dataDescriptionIndex,
                                             std::uint16_t elementNumber,
                                             std::uint8_t errorCodesFromClient,
                                             ProcessDataCommands processDataCommand)
{
	// This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
	std::cout << "[" << get_timestamp() << "] Received process data acknowledge from client " << int(partner->get_address()) << " for DDI " << dataDescriptionIndex << " element " << elementNumber << " with error codes " << std::bitset<8>(errorCodesFromClient) << " and command " << static_cast<int>(processDataCommand) << std::endl;
}

bool MyTCServer::on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
                                  std::uint16_t dataDescriptionIndex,
                                  std::uint16_t elementNumber,
                                  std::int32_t processDataValue,
                                  std::uint8_t &errorCodes)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	switch (dataDescriptionIndex)
	{
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState17_32):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState33_48):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState49_64):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState65_80):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState81_96):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState97_112):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState113_128):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState129_144):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState145_160):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState161_176):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState177_192):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState193_208):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState209_224):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState225_240):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256):
		{
			std::uint8_t sectionIndexOffset = NUMBER_SECTIONS_PER_CONDENSED_MESSAGE * static_cast<std::uint8_t>(dataDescriptionIndex - static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16));

			for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
			{
				std::uint8_t sectionState = ((processDataValue >> (2 * i)) & 0x03);
				clients[partner].set_section_actual_state(i + sectionIndexOffset, sectionState);
				clients[partner].set_element_number_for_section(i + sectionIndexOffset, elementNumber);
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
		{
			clients[partner].set_section_control_enabled(processDataValue == 1);
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
		{
			// Store the work state per element (used for parent-off checks)
			clients[partner].set_element_work_state(elementNumber, processDataValue == 1);

			// For legacy per-element devices only: propagate to section actual states
			// so the heartbeat (PGN 0xF0) can report them to AOG.
			// Modern implements may report both DDI 141 and condensed DDIs (161/290)
			// on the same element — only overwrite section states when per-element
			// control is the active section control method.
			if (clients[partner].uses_per_element_control())
			{
				std::uint8_t sectionIndex;
				if (clients[partner].try_get_section_for_element(elementNumber, sectionIndex))
				{
					clients[partner].set_section_actual_state(sectionIndex, (processDataValue == 1) ? SectionState::ON : SectionState::OFF);
				}
			}
		}
	}

	return true;
}

bool MyTCServer::store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	std::cout << "[" << get_timestamp() << "] [TC Server] Client " << partnerCF->get_NAME().get_full_name() << " requesting object pool transfer of " << binaryPool.size() << " bytes" << std::endl;
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		uploadedPools[partnerCF] = std::queue<std::vector<std::uint8_t>>();
	}
	uploadedPools[partnerCF].push(binaryPool);
	return true;
}

std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> MyTCServer::get_clients()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	return clients; // copy, taken while locked — see the declaration's comment
}

void MyTCServer::request_measurement_commands()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	for (auto &client : clients)
	{
		// Skip clients with 0 sections (e.g. tractors) - sending measurement commands to a tractor ECU can cause unexpected behavior
		if (!client.second.are_measurement_commands_sent() && client.second.get_number_of_sections() > 0)
		{
			// Find all actual (condensed) work state DDIs and request them to trigger "On Change" and "Time Interval"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256)) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::CondensedSectionOverrideState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::CondensedSectionOverrideState241_256)))
					{
						// Loop over all objects to find the elements that are the parents of the actual condensed work state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject && parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());
										std::cout << "Mapped DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
										          << elementObject->get_element_number() << std::endl;

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											std::cout << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											          << elementObject->get_element_number() << std::endl;
										}
										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval))
										{
											send_time_interval_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1000);
										}
									}
								}
							}
						}
					}
				}
			}

			// Find all section control state DDIs and request them to trigger "On Change"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState) ||
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256)))
					{
						// Loop over all objects to find the elements that are the parents of the section control state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject && parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											std::cout << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											          << elementObject->get_element_number() << std::endl;
										}
										else
										{
											std::cout << "Mapped (no OnChange) DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
											          << elementObject->get_element_number() << std::endl;
										}
									}
								}
							}
						}
					}
				}
			}

			std::cout << "[" << get_timestamp() << "] Measurement commands sent." << std::endl;
			client.second.mark_measurement_commands_sent();
		}
	}
}

void MyTCServer::update_section_states(std::vector<bool> &sectionStates)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	for (auto &client : clients)
	{
		auto &state = client.second;
		if (!state.is_section_control_enabled())
		{
			// According to standard, the section setpoint states should only be sent when in auto mode
			continue;
		}

		// Skip clients that don't have any sections configured (e.g., tractors or other non-implement devices)
		if (state.get_number_of_sections() == 0)
		{
			continue;
		}

		bool requiresUpdate = false;
		for (std::uint8_t i = 0; i < state.get_number_of_sections(); i++)
		{
			if ((i % NUMBER_SECTIONS_PER_CONDENSED_MESSAGE == 0) && requiresUpdate)
			{
				// Send the previous 16 sections
				std::uint8_t ddiOffset = (i / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE) - 1;
				send_section_setpoint_states(client.first, ddiOffset);
				requiresUpdate = false;
			}

			if (i < sectionStates.size())
			{
				if (sectionStates[i] != (state.get_section_setpoint_state(i) == SectionState::ON))
				{
					state.set_section_setpoint_state(i, sectionStates[i] ? SectionState::ON : SectionState::OFF);
					requiresUpdate = true;
				}
			}
		}
		if (requiresUpdate)
		{
			std::uint8_t ddiOffset = (state.get_number_of_sections() - 1) / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
			send_section_setpoint_states(client.first, ddiOffset);
		}
	}
}

void MyTCServer::update_section_control_enabled(bool enabled)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	for (auto &client : clients)
	{
		// Always update the local flag
		if (client.second.is_section_control_enabled() != enabled)
		{
			client.second.set_section_control_enabled(enabled);
		}

		// Only send ISOBUS command to clients that support SectionControlState DDI and have sections
		if (client.second.has_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState) &&
		    client.second.get_number_of_sections() > 0)
		{
			send_section_control_state(client.first, enabled);
		}
	}
}

void MyTCServer::send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
	std::uint32_t value = 0;
	for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
	{
		value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
	}

	// Modern ECU? (DDI 290  SetpointCondensedWorkState1_16 exists)
	std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset;
	// Legacy ECU? (DDI 161  ActualCondensedWorkState1_16 exists and Settable)
	std::uint16_t ddiTargetLegacy = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) + ddiOffset;
	if (clients[client].has_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)))
	{
		std::uint16_t elementNumber = clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget));
		send_set_value(client, ddiTarget, elementNumber, value);

		bool setpointWorkState = clients[client].is_any_section_setpoint_on();
		if ((clients[client].get_setpoint_work_state() != setpointWorkState) && clients[client].has_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState))
		{
			send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState), setpointWorkState ? 1 : 0);
			clients[client].set_setpoint_work_state(setpointWorkState);
		}
		else if (!clients[client].has_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState))
		{
			std::cout << "[" << get_timestamp() << "] [TC Server] DDI 289 (SetpointWorkState) not available!" << std::endl;
		}
		return; // Modern condensed path complete
	}
	else if (clients[client].has_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTargetLegacy)))
	{
		if (is_ddi_settable(client, ddiTargetLegacy))
		{
			send_set_value(client, ddiTargetLegacy, clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTargetLegacy)), value);
		}
		else
		{
			std::cout << "[" << get_timestamp() << "] [TC Server] Legacy DDI " << ddiTargetLegacy << " (ActualCondensedWorkState) is not settable!" << std::endl;
		}
		return; // Legacy condensed path complete
	}
	else if (clients[client].uses_per_element_control())
	{
		// Per-element control: send setpoints to each section element individually (DDI 141 settable)
		std::uint16_t setpointDDI = clients[client].get_per_element_setpoint_ddi();
		for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
		{
			std::uint8_t sectionIndex = sectionOffset + i;
			if (sectionIndex >= clients[client].get_number_of_sections())
			{
				break;
			}
			std::uint16_t elementNumber = clients[client].get_element_number_for_section(sectionIndex);
			if (elementNumber != 0)
			{
				std::uint8_t state = clients[client].get_section_setpoint_state(sectionIndex);
				send_set_value(client, setpointDDI, elementNumber, (state == SectionState::ON) ? 1 : 0);
			}
		}

		// Also send global work state on the boom/device element if DDI 289 is available
		bool setpointWorkState = clients[client].is_any_section_setpoint_on();
		if (clients[client].get_setpoint_work_state() != setpointWorkState)
		{
			if (clients[client].has_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState))
			{
				send_set_value(client,
				               static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState),
				               clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState),
				               setpointWorkState ? 1 : 0);
				clients[client].set_setpoint_work_state(setpointWorkState);
			}
		}
		return; // Per-element path complete
	}
	else
	{
		std::cout << "[" << get_timestamp() << "] [TC Server] No supported method to send section setpoint states! "
		          << "Device has no DDI 290, 161 (settable), or 141 (settable)." << std::endl;
	}
}

void MyTCServer::send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
}

bool MyTCServer::is_ddi_settable(std::shared_ptr<isobus::ControlFunction> client, std::uint16_t ddi)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	for (std::uint32_t i = 0; i < clients[client].get_pool().size(); i++)
	{
		auto object = clients[client].get_pool().get_object_by_index(i);
		if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
		{
			auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
			if (processDataObject->get_ddi() == ddi)
			{
				return processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable);
			}
		}
	}
	return false;
}
