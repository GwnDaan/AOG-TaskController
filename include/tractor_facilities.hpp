/**
 * @brief ISO 11783-7 Tractor Facilities (PGN 65033) and Required Tractor
 *        Facilities (PGN 65032) support.
 *
 * Builds the 8-byte facility payload from the TECU's live broadcast state
 * and registers a PGN-request callback so that implements can query which
 * tractor facilities are actually backed by a periodic CAN broadcast.
 *
 * @see ISO 11783-7:2009, B.24.3 (PGN 65033) and B.24.2 (PGN 65032).
 */

#pragma once

#include "isobus/isobus/can_callbacks.hpp"
#include "isobus/isobus/can_control_function.hpp"
#include "isobus/isobus/can_internal_control_function.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <set>

class Settings;

/// @brief Describes which tractor facilities are available.
/// Each boolean maps to a single bit in the 8-byte PGN 65033 payload.
/// Field names follow ISO 11783-7:2009 Table B.24.
struct Facilities
{
	// -- Byte 1 ----------------------------------------------------------
	std::uint8_t tecuClass = 1; ///< TECU class (0-3). 00=Class 1, 01=Class 2, 10=Class 3, 11=N/A.
	bool engineSpeed = false; ///< Speed information – engine speed
	bool groundBasedSpeed = false; ///< Speed information – ground-based speed
	bool wheelBasedSpeed = false; ///< Speed information – wheel-based speed
	bool powerMaintain = false; ///< Power management – maintain power
	bool powerMaxTime = false; ///< Power management – maximum time of tractor power
	bool powerKeySwitch = false; ///< Power management – key switch

	// -- Byte 2 ----------------------------------------------------------
	bool languageCommandStorage = false; ///< Language command storage in Tractor ECU
	bool minimalLighting = false; ///< Lighting – minimal set as existing trailer connector
	bool rearPtoShaftEngagement = false; ///< PTO information – rear shaft engagement
	bool rearPtoShaftSpeed = false; ///< PTO information – rear shaft speed
	bool rearHitchInWork = false; ///< Hitch information – rear in work
	bool rearHitchPosition = false; ///< Hitch information – rear position

	// -- Byte 3 ----------------------------------------------------------
	bool estimatedValveStatus = false; ///< Estimated or measured auxiliary valve status
	bool fullImplementLighting = false; ///< Lighting – full implement lighting message set
	bool rearDraft = false; ///< Additional hitch parameters – rear draft
	bool wheelBasedDirection = false; ///< Speed and distance – wheel-based direction
	bool wheelBasedDistance = false; ///< Speed and distance – wheel-based distance
	bool groundBasedDirection = false; ///< Speed and distance – ground-based direction
	bool groundBasedDistance = false; ///< Speed and distance – ground-based distance
	bool timeDate = false; ///< Time/date

	// -- Byte 4 ----------------------------------------------------------
	bool limitRequestStatusReporting = false; ///< Limit/request status reporting
	bool auxiliaryValveCommands = false; ///< Auxiliary valve commands
	bool rearPtoEngagementCommand = false; ///< PTO commands – rear PTO engagement command
	bool rearPtoSpeedCommand = false; ///< PTO commands – rear PTO speed command
	bool rearHitchPositionCommand = false; ///< Hitch commands – rear hitch position

	// -- Byte 5 ----------------------------------------------------------
	bool directionControl = false; ///< Direction control
	bool selectedSpeedControl = false; ///< Selected speed control
	bool selectedSpeed = false; ///< Selected speed
	bool operatorExternalLightControls = false; ///< Operator external light controls
	// (byte 5 bit 5 is reserved)
	bool navigationalPseudoRangeNoise = false; ///< Navigational pseudo-range noise statistics
	bool navigationalPositionData = false; ///< Navigational system position data
	bool navigationalHighOutputPosition = false; ///< Navigational system high-output position

	// -- Byte 6 ----------------------------------------------------------
	bool frontPtoEngagementCommand = false; ///< PTO commands – front PTO engagement command
	bool frontPtoSpeedCommand = false; ///< PTO commands – front PTO speed command
	bool frontHitchPositionCommand = false; ///< Hitch commands – front hitch position
	bool frontDraft = false; ///< Additional hitch parameters – front draft
	bool frontPtoShaftEngagement = false; ///< PTO information – front shaft engagement
	bool frontPtoShaftSpeed = false; ///< PTO information – front shaft speed
	bool frontHitchInWork = false; ///< Hitch information – front in work
	bool frontHitchPosition = false; ///< Hitch information – front position
};

/// @brief Encode a Facilities struct into the 8-byte PGN 65033 payload.
/// Reserved bits and byte 7 / byte 8 (including the reserved-bit indicator
/// at byte 8 bit 1) are always 0.
std::array<std::uint8_t, 8> encode_facilities(const Facilities &f);

/// @brief Decode an 8-byte PGN 65033 payload into a Facilities struct.
Facilities decode_facilities(const std::array<std::uint8_t, 8> &payload);

/// @brief Implements ISO 11783-7 Tractor Facilities response (PGN 65033)
/// and Required Tractor Facilities diagnostic logging (PGN 65032).
class TractorFacilities
{
public:
	static constexpr std::uint32_t PGN_TRACTOR_FACILITIES = 65033; ///< 0xFE09
	static constexpr std::uint32_t PGN_REQUIRED_TRACTOR_FACILITIES = 65032; ///< 0xFE08

	TractorFacilities(std::shared_ptr<isobus::InternalControlFunction> tecuCF,
	                  std::shared_ptr<Settings> settings);

	/// @brief Register the PGN 65033 request callback and the PGN 65032
	///        global receive listener.  Must be called after the TECU
	///        address claim has completed.
	bool initialize();

	/// @brief Broadcast PGN 65033 once, either as the unsolicited power-up
	///        transmission or in response to a PGN 65033 request.
	/// @param isPowerUp Only affects the wording of the confirmation log line.
	bool send_facilities_response(bool isPowerUp = true);

	/// @brief Provide a pointer to the speed messages interface so the
	///        payload builder can check which speed PGNs are actively
	///        being broadcast.
	void set_speed_messages_interface(isobus::SpeedMessagesInterface *iface);

	/// @brief Provide a pointer to the NMEA 2000 message interface.
	void set_nmea2000_message_interface(isobus::NMEA2000MessageInterface *iface);

	/// @brief Set whether time/date (PGN 65254 / FEE6) is being broadcast.
	/// When true, the time/date facility bit is advertised in PGN 65033.
	void set_time_date_active(bool active);

	/// @brief Build the 8-byte payload from the current runtime state.
	std::array<std::uint8_t, 8> build_payload() const;

private:
	/// @brief Static PGN-request callback registered for PGN 65033.
	static bool on_pgn_request(std::uint32_t parameterGroupNumber,
	                           std::shared_ptr<isobus::ControlFunction> requestingControlFunction,
	                           bool &acknowledge,
	                           isobus::AcknowledgementType &acknowledgeType,
	                           void *parentPointer);

	/// @brief Static global-PGN callback for PGN 65032 (Required Tractor
	///        Facilities). Diagnostic only — logs the request unconditionally
	///        (not gated by log level) and never changes our response.
	static void on_required_facilities(const isobus::CANMessage &message, void *parentPointer);

	std::shared_ptr<isobus::InternalControlFunction> tecuCF;
	std::shared_ptr<Settings> settings;
	isobus::SpeedMessagesInterface *speedMessagesInterface = nullptr;
	isobus::NMEA2000MessageInterface *nmea2000MessageInterface = nullptr;
	/// Whether we are broadcasting PGN 65254 (FEE6). Atomic because
	/// set_time_date_active() and build_payload() are called from different
	/// isobus callback contexts (see docs/CONCURRENCY.md) that can run
	/// concurrently on different threads.
	std::atomic<bool> timeDateActive{ false };

	/// Source addresses for which we have already logged an info-level
	/// "request received" line, to avoid per-message spam.
	std::set<std::uint8_t> loggedRequesters;
};
