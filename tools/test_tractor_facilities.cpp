/**
 * @brief Unit tests for the ISO 11783-7 Tractor Facilities (PGN 65033)
 *        encode / decode logic.
 *
 * Returns 0 when every assertion passes, 1 otherwise.
 */

#include "tractor_facilities.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int failures = 0;

static void check(bool condition, const char *label)
{
	if (!condition)
	{
		std::fprintf(stderr, "  FAIL: %s\n", label);
		++failures;
	}
}

// ---------------------------------------------------------------------------
// Test 1: Default configuration – ground-based speed only
// ---------------------------------------------------------------------------
static void test_default_ground_speed_only()
{
	std::printf("test_default_ground_speed_only\n");

	Facilities f;
	f.tecuClass = 1;
	f.groundBasedSpeed = true;
	f.groundBasedDistance = true;
	f.groundBasedDirection = true;

	auto payload = encode_facilities(f);

	// Byte 1: TECU class 1 → bits 8,7 = 00; ground-based speed (ISO bit 2) → C++ bit 1
	// → 0b00000010 = 0x02
	check(payload[0] == 0x02, "byte 1 = 0x02 (class 1 + ground-based speed)");

	// Byte 3: ground-based distance (ISO bit 7 = C++ bit 6 = 0x40)
	//         + ground-based direction (ISO bit 6 = C++ bit 5 = 0x20)
	// → 0b01100000 = 0x60
	check(payload[2] == 0x60, "byte 3 = 0x60 (ground-based distance + direction)");

	// All other bytes must be 0
	check(payload[1] == 0x00, "byte 2 = 0x00");
	check(payload[3] == 0x00, "byte 4 = 0x00");
	check(payload[4] == 0x00, "byte 5 = 0x00");
	check(payload[5] == 0x00, "byte 6 = 0x00");
	check(payload[6] == 0x00, "byte 7 = 0x00 (reserved)");
	check(payload[7] == 0x00, "byte 8 = 0x00 (reserved + reserved-bit indicator)");
}

// ---------------------------------------------------------------------------
// Test 2: Full default – ground + wheel speed (our TECU's actual default)
// ---------------------------------------------------------------------------
static void test_full_default()
{
	std::printf("test_full_default\n");

	Facilities f;
	f.tecuClass = 1;
	f.groundBasedSpeed = true;
	f.groundBasedDistance = true;
	f.groundBasedDirection = true;
	f.wheelBasedSpeed = true;
	f.wheelBasedDistance = true;
	f.wheelBasedDirection = true;

	auto payload = encode_facilities(f);

	// Byte 1: class 1 (00) + wheel-based (ISO bit 3 = C++ bit 2 = 0x04)
	//         + ground-based (ISO bit 2 = C++ bit 1 = 0x02)
	// → 0b00000110 = 0x06
	check(payload[0] == 0x06, "byte 1 = 0x06 (class 1 + wheel + ground speed)");

	// Byte 3: ground dist (ISO bit 7 = 0x40) + ground dir (ISO bit 6 = 0x20)
	//         + wheel dist (ISO bit 5 = 0x10) + wheel dir (ISO bit 4 = 0x08)
	// → 0b01111000 = 0x78
	check(payload[2] == 0x78, "byte 3 = 0x78 (ground + wheel distance/direction)");

	check(payload[1] == 0x00, "byte 2 = 0x00");
	check(payload[3] == 0x00, "byte 4 = 0x00");
	check(payload[4] == 0x00, "byte 5 = 0x00");
	check(payload[5] == 0x00, "byte 6 = 0x00");
	check(payload[6] == 0x00, "byte 7 = 0x00");
	check(payload[7] == 0x00, "byte 8 = 0x00");
}

// ---------------------------------------------------------------------------
// Test 3: Every reserved bit is 0, including byte 8 bit 1
// ---------------------------------------------------------------------------
static void test_reserved_bits_zero()
{
	std::printf("test_reserved_bits_zero\n");

	// Set every non-reserved facility to true.
	Facilities f;
	f.tecuClass = 1;
	f.engineSpeed = true;
	f.groundBasedSpeed = true;
	f.wheelBasedSpeed = true;
	f.powerMaintain = true;
	f.powerMaxTime = true;
	f.powerKeySwitch = true;
	f.rearHitchPosition = true;
	f.rearPtoShaftSpeed = true;
	f.rearPtoShaftEngagement = true;
	f.minimalLighting = true;
	f.languageCommandStorage = true;
	f.timeDate = true;
	f.groundBasedDistance = true;
	f.groundBasedDirection = true;
	f.wheelBasedDistance = true;
	f.wheelBasedDirection = true;
	f.rearDraft = true;
	f.fullImplementLighting = true;
	f.estimatedValveStatus = true;
	f.rearHitchPositionCommand = true;
	f.rearPtoSpeedCommand = true;
	f.rearPtoEngagementCommand = true;
	f.auxiliaryValveCommands = true;
	f.limitRequestStatusReporting = true;
	f.navigationalHighOutputPosition = true;
	f.navigationalPositionData = true;
	f.navigationalPseudoRangeNoise = true;
	f.operatorExternalLightControls = true;
	f.selectedSpeed = true;
	f.selectedSpeedControl = true;
	f.directionControl = true;
	f.frontHitchPosition = true;
	f.frontPtoShaftSpeed = true;
	f.frontPtoShaftEngagement = true;
	f.frontDraft = true;
	f.frontHitchPositionCommand = true;
	f.frontPtoSpeedCommand = true;
	f.frontPtoEngagementCommand = true;

	auto payload = encode_facilities(f);

	// Byte 2 bits 2,1 are reserved → must be 0
	check((payload[1] & 0x03) == 0x00, "byte 2 bits 2,1 reserved = 0");

	// Byte 4 bits 3-1 are reserved → must be 0
	check((payload[3] & 0x07) == 0x00, "byte 4 bits 3-1 reserved = 0");

	// Byte 5 bit 5 is reserved → must be 0
	check((payload[4] & 0x10) == 0x00, "byte 5 bit 5 reserved = 0");

	// Byte 7 is entirely reserved → must be 0
	check(payload[6] == 0x00, "byte 7 entirely reserved = 0x00");

	// Byte 8 is entirely reserved (including bit 1 reserved-bit indicator) → must be 0
	check(payload[7] == 0x00, "byte 8 entirely reserved = 0x00 (including reserved-bit indicator)");

	// TECU class bits should be 00 (class 1) in bits 8,7
	check((payload[0] & 0xC0) == 0x00, "TECU class = 00 (class 1) in byte 1 bits 8,7");
}

// ---------------------------------------------------------------------------
// Test 4: Disabled broadcast clears its facility bit
// ---------------------------------------------------------------------------
static void test_disabled_broadcast_clears_bit()
{
	std::printf("test_disabled_broadcast_clears_bit\n");

	// Start with ground-based speed enabled.
	Facilities f;
	f.tecuClass = 1;
	f.groundBasedSpeed = true;
	f.groundBasedDistance = true;
	f.groundBasedDirection = true;

	auto payloadEnabled = encode_facilities(f);
	check(payloadEnabled[0] == 0x02, "ground-based speed bit set when enabled");
	check(payloadEnabled[2] == 0x60, "ground-based distance/direction bits set when enabled");

	// Now disable ground-based speed.
	f.groundBasedSpeed = false;
	f.groundBasedDistance = false;
	f.groundBasedDirection = false;

	auto payloadDisabled = encode_facilities(f);
	check(payloadDisabled[0] == 0x00, "ground-based speed bit cleared when disabled");
	check(payloadDisabled[2] == 0x00, "ground-based distance/direction bits cleared when disabled");
}

// ---------------------------------------------------------------------------
// Test 5: Round-trip encode → decode
// ---------------------------------------------------------------------------
static void test_round_trip()
{
	std::printf("test_round_trip\n");

	Facilities original;
	original.tecuClass = 2;
	original.engineSpeed = true;
	original.groundBasedSpeed = true;
	original.wheelBasedSpeed = false;
	original.powerKeySwitch = true;
	original.rearHitchPosition = true;
	original.languageCommandStorage = true;
	original.timeDate = true;
	original.groundBasedDistance = true;
	original.wheelBasedDirection = true;
	original.rearDraft = true;
	original.fullImplementLighting = true;
	original.rearHitchPositionCommand = true;
	original.auxiliaryValveCommands = true;
	original.navigationalPositionData = true;
	original.selectedSpeed = true;
	original.directionControl = true;
	original.frontHitchPosition = true;
	original.frontPtoShaftEngagement = true;
	original.frontPtoSpeedCommand = true;

	auto payload = encode_facilities(original);
	Facilities decoded = decode_facilities(payload);

	check(decoded.tecuClass == original.tecuClass, "tecuClass round-trip");
	check(decoded.engineSpeed == original.engineSpeed, "engineSpeed round-trip");
	check(decoded.groundBasedSpeed == original.groundBasedSpeed, "groundBasedSpeed round-trip");
	check(decoded.wheelBasedSpeed == original.wheelBasedSpeed, "wheelBasedSpeed round-trip");
	check(decoded.powerKeySwitch == original.powerKeySwitch, "powerKeySwitch round-trip");
	check(decoded.powerMaxTime == original.powerMaxTime, "powerMaxTime round-trip");
	check(decoded.powerMaintain == original.powerMaintain, "powerMaintain round-trip");
	check(decoded.rearHitchPosition == original.rearHitchPosition, "rearHitchPosition round-trip");
	check(decoded.rearHitchInWork == original.rearHitchInWork, "rearHitchInWork round-trip");
	check(decoded.rearPtoShaftSpeed == original.rearPtoShaftSpeed, "rearPtoShaftSpeed round-trip");
	check(decoded.rearPtoShaftEngagement == original.rearPtoShaftEngagement, "rearPtoShaftEngagement round-trip");
	check(decoded.minimalLighting == original.minimalLighting, "minimalLighting round-trip");
	check(decoded.languageCommandStorage == original.languageCommandStorage, "languageCommandStorage round-trip");
	check(decoded.timeDate == original.timeDate, "timeDate round-trip");
	check(decoded.groundBasedDistance == original.groundBasedDistance, "groundBasedDistance round-trip");
	check(decoded.groundBasedDirection == original.groundBasedDirection, "groundBasedDirection round-trip");
	check(decoded.wheelBasedDistance == original.wheelBasedDistance, "wheelBasedDistance round-trip");
	check(decoded.wheelBasedDirection == original.wheelBasedDirection, "wheelBasedDirection round-trip");
	check(decoded.rearDraft == original.rearDraft, "rearDraft round-trip");
	check(decoded.fullImplementLighting == original.fullImplementLighting, "fullImplementLighting round-trip");
	check(decoded.estimatedValveStatus == original.estimatedValveStatus, "estimatedValveStatus round-trip");
	check(decoded.rearHitchPositionCommand == original.rearHitchPositionCommand, "rearHitchPositionCommand round-trip");
	check(decoded.rearPtoSpeedCommand == original.rearPtoSpeedCommand, "rearPtoSpeedCommand round-trip");
	check(decoded.rearPtoEngagementCommand == original.rearPtoEngagementCommand, "rearPtoEngagementCommand round-trip");
	check(decoded.auxiliaryValveCommands == original.auxiliaryValveCommands, "auxiliaryValveCommands round-trip");
	check(decoded.limitRequestStatusReporting == original.limitRequestStatusReporting, "limitRequestStatusReporting round-trip");
	check(decoded.navigationalHighOutputPosition == original.navigationalHighOutputPosition, "navigationalHighOutputPosition round-trip");
	check(decoded.navigationalPositionData == original.navigationalPositionData, "navigationalPositionData round-trip");
	check(decoded.navigationalPseudoRangeNoise == original.navigationalPseudoRangeNoise, "navigationalPseudoRangeNoise round-trip");
	check(decoded.operatorExternalLightControls == original.operatorExternalLightControls, "operatorExternalLightControls round-trip");
	check(decoded.selectedSpeed == original.selectedSpeed, "selectedSpeed round-trip");
	check(decoded.selectedSpeedControl == original.selectedSpeedControl, "selectedSpeedControl round-trip");
	check(decoded.directionControl == original.directionControl, "directionControl round-trip");
	check(decoded.frontHitchPosition == original.frontHitchPosition, "frontHitchPosition round-trip");
	check(decoded.frontHitchInWork == original.frontHitchInWork, "frontHitchInWork round-trip");
	check(decoded.frontPtoShaftSpeed == original.frontPtoShaftSpeed, "frontPtoShaftSpeed round-trip");
	check(decoded.frontPtoShaftEngagement == original.frontPtoShaftEngagement, "frontPtoShaftEngagement round-trip");
	check(decoded.frontDraft == original.frontDraft, "frontDraft round-trip");
	check(decoded.frontHitchPositionCommand == original.frontHitchPositionCommand, "frontHitchPositionCommand round-trip");
	check(decoded.frontPtoSpeedCommand == original.frontPtoSpeedCommand, "frontPtoSpeedCommand round-trip");
	check(decoded.frontPtoEngagementCommand == original.frontPtoEngagementCommand, "frontPtoEngagementCommand round-trip");
}

// ---------------------------------------------------------------------------
// Test 6: TECU class not available encoding
// ---------------------------------------------------------------------------
static void test_tecu_class_not_available()
{
	std::printf("test_tecu_class_not_available\n");

	Facilities f;
	f.tecuClass = 0; // 0 = not available (bit pattern 11)

	auto payload = encode_facilities(f);
	// Byte 1 bits 8,7 = 11 → 0b11000000 = 0xC0
	check(payload[0] == 0xC0, "byte 1 = 0xC0 (TECU class not available)");
}

// ---------------------------------------------------------------------------
// Test 7: All-zero payload decodes to default Facilities
// ---------------------------------------------------------------------------
static void test_zero_payload()
{
	std::printf("test_zero_payload\n");

	std::array<std::uint8_t, 8> zero{};
	Facilities f = decode_facilities(zero);

	check(f.tecuClass == 1, "tecuClass = 1 (class 1) from zero payload");
	check(!f.engineSpeed, "engineSpeed = false from zero payload");
	check(!f.groundBasedSpeed, "groundBasedSpeed = false from zero payload");
	check(!f.wheelBasedSpeed, "wheelBasedSpeed = false from zero payload");
	check(!f.rearHitchInWork, "rearHitchInWork = false from zero payload");
	check(!f.languageCommandStorage, "languageCommandStorage = false from zero payload");
	check(!f.timeDate, "timeDate = false from zero payload");
	check(!f.selectedSpeed, "selectedSpeed = false from zero payload");
	check(!f.navigationalPositionData, "navigationalPositionData = false from zero payload");
	check(!f.frontHitchPosition, "frontHitchPosition = false from zero payload");
}

// ---------------------------------------------------------------------------
// Test 8: Rear hitch in work is NOT set by default (safety check)
// ---------------------------------------------------------------------------
static void test_rear_hitch_in_work_not_set()
{
	std::printf("test_rear_hitch_in_work_not_set\n");

	// Even with everything "on" that we support, rear hitch in work must
	// stay 0 unless explicitly requested.
	Facilities f;
	f.tecuClass = 1;
	f.groundBasedSpeed = true;
	f.wheelBasedSpeed = true;

	auto payload = encode_facilities(f);
	// Byte 2 bit 7 = rear hitch in work → must be 0
	check((payload[1] & 0x40) == 0x00, "rear hitch in work (byte 2 bit 7) = 0 by default");
}

int main()
{
	test_default_ground_speed_only();
	test_full_default();
	test_reserved_bits_zero();
	test_disabled_broadcast_clears_bit();
	test_round_trip();
	test_tecu_class_not_available();
	test_zero_payload();
	test_rear_hitch_in_work_not_set();

	if (failures > 0)
	{
		std::fprintf(stderr, "\n%d test(s) FAILED\n", failures);
		return 1;
	}

	std::printf("\nAll tests passed.\n");
	return 0;
}
