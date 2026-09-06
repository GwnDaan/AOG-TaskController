/**
 * @file guidance_track_context.hpp
 * @brief Abstraction layer between AOG input and ISOBUS TRACK (Generation 1) protocol.
 *
 * This header defines the GuidanceTrackContext struct and the GuidanceTrackProvider
 * that consumes AOG PGN 0xF4 guidance-track data.
 *
 * The ISOBUS TRACK sender consumes a GuidanceTrackContext without caring how it
 * was produced. The GuidanceTrackContext maps directly to ISOBUS DDIs 508–511.
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <optional>
#include <span>

#include "logging_utils.hpp"

/// @brief Decode a little-endian uint16 from a 2-byte span starting at offset.
inline std::uint16_t decode_le_u16(std::span<const std::uint8_t> data, std::size_t offset)
{
	return static_cast<std::uint16_t>(data[offset]) |
	  (static_cast<std::uint16_t>(data[offset + 1]) << 8);
}

/// @brief Decode a little-endian, signed int16 from a 2-byte span starting at offset.
inline std::int16_t decode_le_i16(std::span<const std::uint8_t> data, std::size_t offset)
{
	return static_cast<std::int16_t>(decode_le_u16(data, offset));
}

/**
 * @brief Immutable snapshot of guidance-track state consumed by the ISOBUS TRACK sender.
 *
 * Corresponds to ISOBUS DDIs:
 *   - guidanceReferenceLineId -> DDI 508 (Unique A-B Guidance Reference Line ID)
 *   - actualTrackNumber       -> DDI 509 (Actual Track Number)
 *   - trackNumberRight        -> DDI 510 (Track Number to the Right)
 *   - trackNumberLeft         -> DDI 511 (Track Number to the Left)
 */
struct GuidanceTrackContext
{
	std::uint32_t guidanceReferenceLineId = 1; ///< DDI 508 — stable synthetic ID for now
	std::int32_t actualTrackNumber = 0; ///< DDI 509 — signed; track 0 is valid
	std::int32_t trackNumberRight = -1; ///< DDI 510
	std::int32_t trackNumberLeft = 1; ///< DDI 511
	bool valid = false; ///< Context has been initialized with at least one update
};

/**
 * @brief Real guidance-track provider that consumes AOG PGN 0xF4 (244) data.
 *
 * PGN 0xF4 payload layout (10 data bytes):
 *   Byte 0: Sequence counter (0–255, wrapping)
 *   Byte 1: Flags (bit 0 = valid, bit 1 = heading same way, bit 2 = curve mode)
 *   Bytes 2-3: Guidance Reference ID (uint16 LE)
 *   Bytes 4-5: Current Track Number (int16 LE, signed)
 *   Bytes 6-7: Track Number Left (int16 LE, signed)
 *   Bytes 8-9: Track Number Right (int16 LE, signed)
 *
 * Track numbers (current/left/right) are each shifted by +1 from the raw wire value
 * (see AOG_TRACK_NUMBER_OFFSET in parse()) to match the tram-pattern phase implements
 * expect — confirmed by field testing, not part of AOG's own documented wire format.
 *
 * Sequence tracking:
 *   - Rejects any packet whose sequence number is not strictly ahead of the last
 *     accepted one (signed delta over the 0-255 wrap), catching both frozen/duplicate
 *     data and reordered/stale UDP packets arriving out of order.
 *   - Call reset() after a disconnect to treat the next packet as a fresh start.
 *
 * Returns valid=true when data is valid, refId != 0, and sequence is fresh.
 */
class GuidanceTrackProvider
{
public:
	/// Minimum payload size (10 data bytes)
	static constexpr std::size_t MIN_PAYLOAD_SIZE = 10;

	/**
	 * @brief Parse AOG PGN 0xF4 payload and produce a GuidanceTrackContext.
	 *
	 * @param data Payload bytes (after UDP header stripping)
	 * @return GuidanceTrackContext with valid=true if parse succeeded and flags indicate valid data
	 */
	GuidanceTrackContext parse(std::span<const std::uint8_t> data)
	{
		GuidanceTrackContext ctx;

		if (data.size() < MIN_PAYLOAD_SIZE)
		{
			std::cout << "[" << get_timestamp() << "] [TRACK][real] PGN 0xF4 too short (len="
			          << data.size() << ")" << std::endl;
			return ctx;
		}

		// Parse fields
		std::uint8_t sequence = data[0];
		std::uint8_t flags = data[1];
		bool isValid = (flags & 0x01) != 0;
		bool headingSameWay = (flags & 0x02) != 0;
		bool curveMode = (flags & 0x04) != 0;

		std::uint16_t refId = decode_le_u16(data, 2);

		// AOG's raw track-number wire convention is one pass off from what a tramline
		// implement's own on-board phase computation (e.g. "which of N passes is this")
		// expects, for both left and right passes. Confirmed via field testing: with the
		// raw value relayed as-is, the sprayer's ON/OFF tram state was consistently one
		// pass out of phase with AOG's own intended tram state; shifting every value by
		// +1 (independent of sign) brought them into agreement across every pass tested.
		static constexpr std::int16_t AOG_TRACK_NUMBER_OFFSET = 1;
		std::int16_t currentTrack = static_cast<std::int16_t>(decode_le_i16(data, 4) + AOG_TRACK_NUMBER_OFFSET);
		std::int16_t trackLeft = static_cast<std::int16_t>(decode_le_i16(data, 6) + AOG_TRACK_NUMBER_OFFSET);
		std::int16_t trackRight = static_cast<std::int16_t>(decode_le_i16(data, 8) + AOG_TRACK_NUMBER_OFFSET);

		// Sequence freshness check: signed delta over the 0-255 wrap must be strictly
		// positive (forward progress). Rejects both frozen/duplicate packets (delta == 0)
		// and reordered/stale packets that arrived out of order (delta < 0).
		const char *outcome;

		if (lastSequence_.has_value() && static_cast<std::int8_t>(sequence - *lastSequence_) <= 0)
		{
			outcome = "REJECTED (stale/duplicate/out-of-order sequence)";
		}
		else
		{
			lastSequence_ = sequence;

			if (!isValid || refId == 0)
			{
				outcome = "guidance OFF (no active track)";
			}
			else
			{
				ctx.guidanceReferenceLineId = refId;
				ctx.actualTrackNumber = currentTrack;
				ctx.trackNumberLeft = trackLeft;
				ctx.trackNumberRight = trackRight;
				ctx.valid = true;
				outcome = "ACCEPTED";
			}
		}

		std::cout << "[" << get_timestamp() << "] [TRACK][real] seq=" << static_cast<int>(sequence)
		          << " flags=0x" << std::hex << static_cast<int>(flags) << std::dec
		          << " (valid=" << isValid << " sameHeading=" << headingSameWay << " curve=" << curveMode << ")"
		          << " ref=" << refId
		          << " left=" << trackLeft << " actual=" << currentTrack << " right=" << trackRight
		          << " -> " << outcome << std::endl;

		return ctx;
	}

	/// @brief Reset sequence tracking (e.g., after AOG disconnect timeout).
	/// The next parse() call is treated as a fresh start — no delta comparison.
	void reset()
	{
		lastSequence_.reset();
	}

private:
	std::optional<std::uint8_t> lastSequence_; ///< Unset until the first packet is accepted
};
