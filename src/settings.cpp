/**
 * @author Daan Steenbergen
 * @brief An interface to store/load AOG-TC settings to/from a file
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "settings.hpp"

#include <ShlObj_core.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

bool Settings::load()
{
	std::ifstream file(get_filename_path("settings.json"));
	if (!file.is_open())
	{
		return false;
	}

	json data;
	file >> data;

	if (data.contains("subnet"))
	{
		try
		{
			auto subnetData = data["subnet"].get<std::array<int, 3>>(); // Directly get the array
			std::copy(subnetData.begin(), subnetData.end(), configuredSubnet.begin());
		}
		catch (const nlohmann::json::exception &e)
		{
			std::cout << "Error parsing 'subnet': " << e.what() << std::endl;
			configuredSubnet = DEFAULT_SUBNET; // Fallback to default
		}
	}
	else
	{
		configuredSubnet = DEFAULT_SUBNET; // Key not found, use default
	}

	return true;
}

bool Settings::save() const
{
	json data;
	data["subnet"] = configuredSubnet;

	std::ofstream file(get_filename_path("settings.json"));
	if (!file.is_open())
	{
		return false;
	}

	file << data.dump(4); // Pretty print
	return true;
}

const std::array<std::uint8_t, 3> &Settings::get_subnet() const
{
	return configuredSubnet;
}

std::string Settings::get_subnet_string() const
{
	return std::to_string(configuredSubnet[0]) + '.' + std::to_string(configuredSubnet[1]) + '.' + std::to_string(configuredSubnet[2]) + ".0";
}

bool Settings::set_subnet(std::array<std::uint8_t, 3> subnet, bool save)
{
	configuredSubnet = subnet;
	if (save)
	{
		return this->save();
	}
	return true;
}

std::string Settings::get_filename_path(std::string fileName)
{
	char path[MAX_PATH];
	if (SHGetFolderPath(NULL, CSIDL_APPDATA, NULL, 0, path) != S_OK)
	{
		throw std::runtime_error("Failed to get AppData path");
	}

	std::string baseDir = std::string(path) + "\\" + PROJECT_NAME;
	std::string fullPath = baseDir + "\\" + fileName;

	// Find the last directory separator (before the actual file name)
	size_t lastSlash = fullPath.find_last_of("\\/");
	if (lastSlash != std::string::npos)
	{
		std::string directoryPath = fullPath.substr(0, lastSlash); // Extract the directory part

		// Create each directory level iteratively
		std::istringstream dirStream(directoryPath);
		std::string segment;
		std::string currentPath;

		while (std::getline(dirStream, segment, '\\')) // Split by `\`
		{
			if (!currentPath.empty())
				currentPath += "\\"; // Append separator only after first segment

			currentPath += segment;

			if (CreateDirectory(currentPath.c_str(), NULL) == 0)
			{
				DWORD error = GetLastError();
				if (error != ERROR_ALREADY_EXISTS)
				{
					throw std::runtime_error("Failed to create directory: " + currentPath);
				}
			}
		}
	}

	return fullPath;
}
