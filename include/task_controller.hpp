/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include "guidance_track_context.hpp"
#include "isobus/isobus/isobus_data_dictionary.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <queue>

constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;

enum SectionState : std::uint8_t
{
	OFF = 0, ///< Section is off
	ON = 1, ///< Section is on
	ERROR_SATE = 2, ///< Section is in an error state
	NOT_INSTALLED = 3 ///< Section is not installed
};

// Tramline control level bitmask (ISO 11783-10)
// An implement may support any combination of levels.
enum class TramlineLevel : std::uint8_t
{
	None = 0,
	Level1 = 1, ///< Track info: ActualTrackNumber, adjacent tracks, swath width
	Level2 = 2, ///< Control state: TramlineControlState, SequenceNumber
	Level3 = 4 ///< Direct valve control: SetpointTramlineCondensedWorkState
};

class ClientState
{
public:
	void set_number_of_sections(std::uint8_t number);
	void set_section_setpoint_state(std::uint8_t section, std::uint8_t state);
	void set_section_actual_state(std::uint8_t section, std::uint8_t state);
	std::uint8_t get_number_of_sections() const;
	std::uint8_t get_section_setpoint_state(std::uint8_t section) const;
	std::uint8_t get_section_actual_state(std::uint8_t section) const;
	std::uint16_t get_element_number_for_section(std::uint8_t section) const;
	void set_element_number_for_section(std::uint8_t section, std::uint16_t elementNumber);
	bool try_get_section_for_element(std::uint16_t elementNumber, std::uint8_t &section) const;
	bool is_any_section_setpoint_on() const;
	bool get_setpoint_work_state() const;
	void set_setpoint_work_state(bool state);
	bool get_actual_work_state() const;
	void set_actual_work_state(bool state);
	bool is_section_control_enabled() const;
	void set_section_control_enabled(bool state);
	bool uses_per_element_control() const;
	void set_uses_per_element_control(bool state);
	std::uint16_t get_per_element_setpoint_ddi() const;
	void set_per_element_setpoint_ddi(std::uint16_t ddi);
	isobus::DeviceDescriptorObjectPool &get_pool();
	bool are_measurement_commands_sent() const;
	void mark_measurement_commands_sent();
	std::uint16_t get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const;
	void set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber);
	bool has_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const;
	bool is_element_or_parent_off(std::uint16_t elementNumber) const; ///< Recursively checks if element or any parent is off
	// Element work state management these act like master / override for actual sections
	void set_element_work_state(std::uint16_t elementNumber, bool isWorking);
	bool try_get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const;

	// Tramline support — DDI 505 bitmask from implement (raw value, not inferred)
	int get_supported_tramline_levels_bitmask() const;
	void set_supported_tramline_levels_bitmask(int bitmask);

	// Track negotiation state (DDI 505/506 handshake complete)
	bool is_track_negotiation_complete() const;
	void set_track_negotiation_complete(bool complete);

	// Track control enabled (separate from section control, Requirement 7)
	bool is_track_control_enabled() const;
	void set_track_control_enabled(bool enabled);

	void set_actual_tramline_control_state(std::int32_t value);
	std::int32_t get_actual_tramline_control_state() const;
	std::uint32_t get_tramline_sequence_number() const;
	void increment_tramline_sequence_number();

	// Last sent track number / reference line ID — used to detect changes for DDI 507 sequence increment
	std::int32_t get_last_sent_track_number() const;
	void set_last_sent_track_number(std::int32_t trackNumber);
	std::uint32_t get_last_sent_reference_line_id() const;
	void set_last_sent_reference_line_id(std::uint32_t referenceLineId);

	// DDI 505/506 presence flags
	bool get_has_tramline_control_level() const;
	void set_has_tramline_control_level(bool has);
	bool get_has_setpoint_tramline_control_level() const;
	void set_has_setpoint_tramline_control_level(bool has);
	bool is_setpoint_level_sent() const;
	void set_setpoint_level_sent(bool sent);

	// Working width reported live over the bus (DDI 67/68/70). Needed because these DDIs
	// are commonly implemented as Device Process Data (DPD) rather than Device Property
	// (DPT) — a DPD has no value embedded in the DDOP itself (see isobus's
	// DeviceProcessDataObject, which has no value field/getter at all), only a definition.
	// The real value only ever arrives as a live process data value message, which is why
	// DeviceDescriptorObjectPoolHelper::get_implement_geometry() (a pure static-pool parser)
	// can never see it. We store what the client actually reports here instead.
	void set_reported_working_width(std::uint16_t elementNumber, isobus::DataDescriptionIndex ddi, std::int32_t widthMm);
	bool try_get_reported_working_width(std::uint16_t elementNumber, std::int32_t &widthMm) const;

private:
	isobus::DeviceDescriptorObjectPool pool; ///< The device descriptor object pool (DDOP) for the TC
	bool areMeasurementCommandsSent = false; ///< Whether or not the measurement commands have been sent
	std::map<isobus::DataDescriptionIndex, std::uint16_t> ddiToElementNumber; ///< Mapping of DDI to element number // TODO: better way to do this?

	std::uint8_t numberOfSections;
	std::vector<std::uint8_t> sectionSetpointStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	std::vector<std::uint8_t> sectionActualStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	std::vector<std::uint16_t> sectionToElementNumber; // Maps section index to element number for hierarchy checking
	std::map<std::uint16_t, std::uint8_t> elementToSection; ///< Reverse mapping: element number -> section index
	bool setpointWorkState = false; ///< The overall work state desired (DDI 289)
	bool actualWorkState = false; ///< The overall work state actual
	std::map<std::uint16_t, bool> elementWorkStates; ///< Work state per element (element number -> is working)
	bool isSectionControlEnabled = false; ///< Stores auto vs manual mode setting
	int supportedTramlineLevelsBitmask = 0; ///< Raw DDI 505 bitmask from implement (bit 0=L1, bit 1=L2, bit 2=L3)
	bool trackNegotiationComplete = false; ///< DDI 505/506 handshake completed
	bool trackControlEnabled = false; ///< Track control enabled (separate from section control)
	std::int32_t actualTramlineControlState = 0; ///< Actual tramline control state reported by implement (DDI 0x0203)
	std::int32_t lastSentTrackNumber = 0; ///< Last track number sent, for DDI 507 change detection
	std::uint32_t lastSentReferenceLineId = 0; ///< Last reference line ID sent, for DDI 507 change detection
	std::uint32_t tramlineSequenceNumber = 0; ///< Per-client tramline sequence number (DDI 507)
	bool hasTramlineControlLevelDDI = false; ///< Implement has DDI 505 (TramlineControlLevel)
	bool hasSetpointTramlineControlLevelDDI = false; ///< Implement has DDI 506 (SetpointTramlineControlLevel)
	bool setpointLevelSent = false; ///< Whether we've already written DDI 506
	bool usesPerElementControl = false; ///< Legacy mode: use per-element setpoint instead of condensed
	std::uint16_t perElementSetpointDDI = 0; ///< The DDI to use for per-element setpoints (289 or 141), 0 if not applicable

	/// @brief Live working width reported by the client for one device element (DDI 67/68/70).
	struct ReportedWidth
	{
		bool hasActual = false;
		std::int32_t actual = 0;
		bool hasMaximum = false;
		std::int32_t maximum = 0;
		bool hasDefault = false;
		std::int32_t defaultValue = 0;
	};
	std::map<std::uint16_t, ReportedWidth> elementReportedWidthMm; ///< element number -> reported width(s), same priority scheme as DeviceDescriptorObjectPoolHelper (Actual > Maximum > Default)
};

// Create the task controller server object, this will handle all the ISOBUS communication for us
class MyTCServer : public isobus::TaskControllerServer
{
public:
	MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction,
	           isobus::TaskControllerServer::TaskControllerVersion version = isobus::TaskControllerServer::TaskControllerVersion::SecondEditionDraft);
	bool activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &) override;
	bool change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &) override;
	bool deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF) override;
	bool delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &) override;
	bool get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &) override;
	bool get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &) override;
	bool get_is_enough_memory_available(std::uint32_t) override;
	void identify_task_controller(std::uint8_t) override;
	void on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner) override;
	void on_client_version_received(std::shared_ptr<isobus::ControlFunction> clientControlFunction, std::uint8_t version) override;
	void on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner, std::uint16_t dataDescriptionIndex, std::uint16_t elementNumber, std::uint8_t errorCodesFromClient, ProcessDataCommands processDataCommand) override;
	bool on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
	                      std::uint16_t dataDescriptionIndex,
	                      std::uint16_t elementNumber,
	                      std::int32_t processDataValue,
	                      std::uint8_t &errorCodes) override;
	bool store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool) override;

	/// @brief Returns a snapshot copy of the client map, not a live reference.
	/// See the concurrency note on clientsMutex below for why: the isobus stack
	/// invokes the TaskControllerServer overrides above from its own background
	/// thread, concurrently with whichever thread calls this. A returned reference
	/// could be mutated (even reallocated, on insert/erase) out from under a caller
	/// mid-iteration — a returned copy can't.
	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> get_clients();
	void request_measurement_commands();
	void update_section_states(std::vector<bool> &sectionStates);
	void update_section_control_enabled(bool enabled);
	void update_track_control_enabled(bool enabled);
	void send_tramline_track_data(const GuidanceTrackContext &ctx, std::int32_t swathWidthMm, std::int32_t lineDeviationMm, std::uint8_t gnssFixQuality);

private:
	void send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset);
	void send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled);
	bool is_ddi_settable(std::shared_ptr<isobus::ControlFunction> client, std::uint16_t ddi);

	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> clients;
	std::map<std::shared_ptr<isobus::ControlFunction>, std::queue<std::vector<std::uint8_t>>> uploadedPools;

	/// @brief Guards clients and uploadedPools.
	///
	/// CONCURRENCY: the isobus/AgIsoStack stack runs its own background thread
	/// (CANHardwareInterface's updateThread) that calls CANNetworkManager::update(),
	/// which is what actually invokes every TaskControllerServer override in this
	/// class (activate_object_pool, on_value_command, ...) — NOT the thread that
	/// runs Application::update(). Meanwhile Application::update() (and the methods
	/// it calls: send_tramline_track_data, request_measurement_commands,
	/// update_section_states/_control_enabled, get_clients) reads/writes the SAME
	/// maps from the main thread. Without this lock, that's a concurrent
	/// std::map read + insert/erase — undefined behavior, seen in practice as the
	/// TC silently crashing (no exception, no log line) whenever a new control
	/// function appeared on the bus. See docs/CONCURRENCY.md before touching
	/// clients/uploadedPools or adding a new callback here: every entry point the
	/// isobus stack can call into must take this lock (std::recursive_mutex — some
	/// of these methods call each other, e.g. update_section_states ->
	/// send_section_setpoint_states), and get_clients() must keep returning a copy,
	/// not a reference (see its declaration above).
	mutable std::recursive_mutex clientsMutex;
};
