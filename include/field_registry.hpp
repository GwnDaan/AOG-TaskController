/**
 * @file field_registry.hpp
 * @brief Persistent field-name -> field-index mapping, used to make ISOBUS TRACK's
 * DDI 508 (Unique A-B Guidance Reference Line ID) actually unique across fields.
 *
 * AOG's own PGN 0xF4 guidance reference ID is only a 16-bit value scoped to whatever
 * field is currently open in AOG - it is not guaranteed unique across different fields.
 * An implement that caches per-track state (e.g. an offset) keyed on DDI 508 alone can
 * therefore collide across a field switch. This registry assigns each field name a
 * stable index, which the caller folds into the upper 16 bits of the 32-bit DDI 508
 * value (see Application's PGN 0xF3/0xF4 handling), leaving AOG's own 16-bit ID in the
 * lower 16 bits untouched.
 */

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

/// @brief Loads/persists a field-name -> field-index mapping from a plain text file.
///
/// Deliberately its own file rather than part of settings.json: the mapping only grows
/// over time (one line per field ever opened), so a user may want to wipe it on its own
/// (e.g. to reclaim indices) without touching the rest of their configuration.
class FieldRegistry
{
public:
	/// @brief Loads the registry from disk, if present. A missing or unreadable file
	/// just starts empty - fields get freshly (re-)indexed and persisted as they're seen.
	FieldRegistry();

	/// @brief Returns the persistent index for a field name, assigning and persisting a
	/// new one the first time this name is seen.
	/// @param fieldName UTF-8 field folder name, as received from AOG PGN 0xF3.
	/// @returns A stable index. Once the 16-bit space is exhausted (65536 distinct
	/// field names - far beyond realistic use), the most recently assigned index is
	/// reused and a warning is logged, rather than silently colliding with an existing
	/// field.
	std::uint16_t get_or_assign_index(const std::string &fieldName);

private:
	void load();
	void append_entry(const std::string &fieldName, std::uint16_t index);

	std::string filePath;
	std::unordered_map<std::string, std::uint16_t> nameToIndex;
	std::uint16_t nextIndex = 0;
	bool nextIndexExhausted = false;
};
