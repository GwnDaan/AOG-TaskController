/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2024-12-16
 *
 * @copyright 2024 Daan Steenbergen
 *
 */

#include <atomic>
#include <boost/asio.hpp>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <optional>

#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"
#include "isobus/utility/system_timing.hpp"

#include <bitset>
#include <map>
#include "console_logger.cpp"
#include "settings.hpp"
#include "udp_connections.hpp"

constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;

enum SectionState : std::uint8_t
{
	OFF = 0, ///< Section is off
	ON = 1, ///< Section is on
	ERROR_SATE = 2, ///< Section is in an error state
	NOT_INSTALLED = 3 ///< Section is not installed
};

using boost::asio::ip::udp;

static std::atomic_bool running = { true };
void signal_handler(int)
{
	running = false;
}

auto settings = std::make_shared<Settings>();
auto ioContext = boost::asio::io_context();
auto udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

static std::atomic_bool toggleSectionControl = { false };

void read_auto_mode()
{
	std::cout << "Enter 'T' to toggle section control:" << std::endl;
	std::string input;
	while (running)
	{
		std::getline(std::cin, input);

		if (input == "T" || input == "t")
		{
			toggleSectionControl = true;
		}
		else
		{
			std::cout << "Invalid input, please enter 'T' to toggle section control" << std::endl;
		}
	}
}

class ClientState
{
public:
	void set_number_of_sections(std::uint8_t number)
	{
		numberOfSections = number;
		sectionSetpointStates.resize(number);
		sectionActualStates.resize(number);
	}

	void set_section_setpoint_state(std::uint8_t section, std::uint8_t state)
	{
		if (section < numberOfSections)
		{
			sectionSetpointStates[section] = state;
		}
	}

	void set_section_actual_state(std::uint8_t section, std::uint8_t state)
	{
		if (section < numberOfSections)
		{
			sectionActualStates[section] = state;
		}
	}

	std::uint8_t get_number_of_sections() const
	{
		return numberOfSections;
	}

	std::uint8_t get_section_setpoint_state(std::uint8_t section) const
	{
		if (section < numberOfSections)
		{
			return sectionSetpointStates[section];
		}
		return SectionState::NOT_INSTALLED;
	}

	std::uint8_t get_section_actual_state(std::uint8_t section) const
	{
		if (section < numberOfSections)
		{
			return sectionActualStates[section];
		}
		return SectionState::NOT_INSTALLED;
	}

	bool is_any_section_setpoint_on() const
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

	bool get_setpoint_work_state() const
	{
		return setpointWorkState;
	}

	void set_setpoint_work_state(bool state)
	{
		setpointWorkState = state;
	}

	bool get_actual_work_state() const
	{
		return actualWorkState;
	}

	void set_actual_work_state(bool state)
	{
		actualWorkState = state;
	}

	bool is_section_control_enabled() const
	{
		return isSectionControlEnabled;
	}

	void set_section_control_enabled(bool state)
	{
		isSectionControlEnabled = state;
	}

	isobus::DeviceDescriptorObjectPool &get_pool()
	{
		return pool;
	}

	bool are_measurement_commands_sent() const
	{
		return areMeasurementCommandsSent;
	}

	void mark_measurement_commands_sent()
	{
		areMeasurementCommandsSent = true;
	}

	std::uint16_t get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
	{
		auto it = ddiToElementNumber.find(ddi);
		if (it != ddiToElementNumber.end())
		{
			return it->second;
		}
		std::cerr << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
		return 0;
	}

	void set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
	{
		ddiToElementNumber[ddi] = elementNumber;
	}

private:
	isobus::DeviceDescriptorObjectPool pool; ///< The device descriptor object pool (DDOP) for the TC
	bool areMeasurementCommandsSent = false; ///< Whether or not the measurement commands have been sent
	std::map<isobus::DataDescriptionIndex, std::uint16_t> ddiToElementNumber; ///< Mapping of DDI to element number // TODO: better way to do this?

	std::uint8_t numberOfSections;
	std::vector<std::uint8_t> sectionSetpointStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	std::vector<std::uint8_t> sectionActualStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	bool setpointWorkState = false; ///< The overall work state desired
	bool actualWorkState = false; ///< The overall work state actual
	bool isSectionControlEnabled = false; ///< Stores auto vs manual mode setting
};

// Create the task controller server object, this will handle all the ISOBUS communication for us
class MyTCServer : public isobus::TaskControllerServer
{
public:
	MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction) :
	  TaskControllerServer(internalControlFunction,
	                       1, // AOG limits to 1 boom
	                       16, // AOG limits to 16 sections of unique width
	                       16, // 16 channels for position based control
	                       isobus::TaskControllerOptions()
	                         .with_implement_section_control(), // We support section control
	                       TaskControllerVersion::SecondEditionDraft)
	{
	}

	bool activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &) override
	{
		// Safety check to make sure partnerCF has uploaded a DDOP
		if (uploadedPools.find(partnerCF) == uploadedPools.end())
		{
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

			// std::ofstream outFile("partial-ddop-" + std::to_string(std::time(nullptr)) + ".bin", std::ios::binary);
			// outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
			// outFile.close();
		}
		if (deserialized)
		{
			// std::vector<std::uint8_t> binaryPool;
			// bool success = state.get_pool().generate_binary_object_pool(binaryPool);
			// if (success)
			// {
			// 	std::ofstream outFile("ddop-" + std::to_string(std::time(nullptr)) + ".bin", std::ios::binary);
			// 	outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
			// 	outFile.close();
			// }

			std::cout << "Successfully deserialized device descriptor object pool." << std::endl;
			auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(state.get_pool());
			std::uint8_t numberOfSections = 0;

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
					std::cout << "Section: id=" << static_cast<int>(section.elementNumber) << std::endl;
					std::cout << "X Offset: " << section.xOffset_mm.get() << std::endl;
					std::cout << "Y Offset: " << section.yOffset_mm.get() << std::endl;
					std::cout << "Z Offset: " << section.zOffset_mm.get() << std::endl;
					std::cout << "Width: " << section.width_mm.get() << std::endl;
				}
			}
			state.set_number_of_sections(numberOfSections);
		}
		else
		{
			std::cout << "Failed to deserialize device descriptor object pool." << std::endl;
			return false;
		}

		clients[partnerCF] = state;
		return true;
	}

	bool change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &) override
	{
		return true;
	}

	bool deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF) override
	{
		clients.erase(partnerCF);
		uploadedPools.erase(partnerCF);
		return true;
	}

	bool delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &) override
	{
		clients.erase(partnerCF);
		uploadedPools.erase(partnerCF);
		return true;
	}

	bool get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &) override
	{
		return false;
	}

	bool get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &) override
	{
		return false;
	}

	bool get_is_enough_memory_available(std::uint32_t) override
	{
		return true;
	}

	void identify_task_controller(std::uint8_t) override
	{
		// When this is called, the TC is supposed to display its TC number for 3 seconds if possible (which is passed into this function).
		// Your TC's number is your function code + 1, in the range of 1-32.
	}

	void on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner) override
	{
		// Cleanup the client state
		clients.erase(partner);
	}

	void on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner, std::uint16_t dataDescriptionIndex, std::uint16_t elementNumber, std::uint8_t errorCodesFromClient, ProcessDataCommands processDataCommand) override
	{
		// This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
		std::cout << "Received process data acknowledge from client " << int(partner->get_address()) << " for DDI " << dataDescriptionIndex << " element " << elementNumber << " with error codes " << std::bitset<8>(errorCodesFromClient) << " and command " << static_cast<int>(processDataCommand) << std::endl;
	}

	bool on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
	                      std::uint16_t dataDescriptionIndex,
	                      std::uint16_t elementNumber,
	                      std::int32_t processDataValue,
	                      std::uint8_t &errorCodes) override
	{
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
					clients[partner].set_section_actual_state(i + sectionIndexOffset, (processDataValue >> (2 * i)) & 0x03);
				}

				std::cout << "Received actual condensed work state for element number " << int(elementNumber) << " and DDI " << int(dataDescriptionIndex) << " with states: ";
				for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
				{
					std::cout << static_cast<int>(clients[partner].get_section_actual_state(sectionIndexOffset + i)) << " ";
				}
				std::cout << std::endl;
			}
			break;

			case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
			{
				std::cout << "Received section control state: " << processDataValue << std::endl;
				clients[partner].set_section_control_enabled(processDataValue == 1);
			}
			break;

			case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
			{
				std::cout << "Received actual work state: " << processDataValue << std::endl;
				clients[partner].set_setpoint_work_state(processDataValue == 1);
			}
		}

		return true;
	}

	bool store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool) override
	{
		if (uploadedPools.find(partnerCF) == uploadedPools.end())
		{
			uploadedPools[partnerCF] = std::queue<std::vector<std::uint8_t>>();
		}
		uploadedPools[partnerCF].push(binaryPool);
		return true;
	}

	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &get_clients()
	{
		return clients;
	}

	void request_measurement_commands()
	{
		for (auto &client : clients)
		{
			if (!client.second.are_measurement_commands_sent())
			{
				// Find all actual (condensed) work state DDIs and request them to trigger "On Change" and "Time Interval"
				for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
				{
					auto object = client.second.get_pool().get_object_by_index(i);
					if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
					{
						auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
						if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) ||
						    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
						     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256)))
						{
							// Loop over all objects to find the elements that are the parents of the actual condensed work state objects
							for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
							{
								auto parentObject = client.second.get_pool().get_object_by_index(j);
								if (parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
								{
									auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
									for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
									{
										if (elementObjectChild == processDataObject->get_object_id())
										{
											// TODO: This is a bit of a hack, but it works for now
											client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());

											if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
											{
												std::cout << "Requesting on-change trigger for element number " << int(elementObject->get_element_number()) << " and DDI " << int(processDataObject->get_ddi()) << std::endl;
												send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											}
											if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval))
											{
												std::cout << "Requesting time interval trigger for element number " << int(elementObject->get_element_number()) << " and DDI " << int(processDataObject->get_ddi()) << std::endl;
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
					if (object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
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
								if (parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
								{
									auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
									for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
									{
										if (elementObjectChild == processDataObject->get_object_id())
										{
											// TODO: This is a bit of a hack, but it works for now
											client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());

											if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
											{
												std::cout << "Requesting on-change trigger for element number " << int(elementObject->get_element_number()) << " and DDI " << int(processDataObject->get_ddi()) << std::endl;
												send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											}
										}
									}
								}
							}
						}
					}
				}

				std::cout << "Measurement commands sent." << std::endl;
				client.second.mark_measurement_commands_sent();
			}
		}
	}

	void update_section_states(std::vector<bool> &sectionStates)
	{
		for (auto &client : clients)
		{
			bool requiresUpdate = false;
			auto &state = client.second;
			for (std::uint8_t i = 0; i < state.get_number_of_sections(); i++)
			{
				if ((i % 16 == 0) && requiresUpdate)
				{
					std::uint8_t ddiOffset = i / 16;
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
				std::uint8_t ddiOffset = (state.get_number_of_sections() + 1) / 16;
				send_section_setpoint_states(client.first, ddiOffset);
			}
		}
	}

	void update_section_control_enabled(bool enabled)
	{
		for (auto &client : clients)
		{
			if (client.second.is_section_control_enabled() != enabled)
			{
				client.second.set_section_control_enabled(enabled);
				send_section_control_state(client.first, enabled);
			}
		}
	}

private:
	void send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
	{
		if (!clients[client].is_section_control_enabled())
		{
			// According to standard, the section setpoint states should only be sent when in auto mode
			return;
		}

		std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
		std::uint32_t value = 0;
		for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
		{
			value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
		}

		std::cout << "Sending setpoint states for DDI offset " << static_cast<int>(ddiOffset) << " with states: ";
		for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
		{
			std::cout << static_cast<int>(clients[client].get_section_setpoint_state(sectionOffset + i)) << " ";
		}
		std::cout << std::endl;

		std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset;
		std::uint16_t elementNumber = clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget));
		send_set_value(client, ddiTarget, elementNumber, value);

		bool setpointWorkState = clients[client].is_any_section_setpoint_on();
		if ((clients[client].get_setpoint_work_state() != setpointWorkState))
		{
			std::cout << "Sending setpoint work state: " << (setpointWorkState ? "on" : "off") << std::endl;
			send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState), setpointWorkState ? 1 : 0);
			clients[client].set_setpoint_work_state(setpointWorkState);
		}
	}

	void send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
	{
		std::cout << "Sending section control state: " << (enabled ? "enabled" : "disabled") << std::endl;
		send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
	}

	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> clients;
	std::map<std::shared_ptr<isobus::ControlFunction>, std::queue<std::vector<std::uint8_t>>> uploadedPools;
};

int main()
{
	std::thread inputThread(read_auto_mode); // TODO: read from AOG

	std::signal(SIGINT, signal_handler);

	settings->load();

	auto canDriver = std::make_shared<isobus::NTCANPlugin>(42);
	if (nullptr == canDriver)
	{
		std::cout << "Unable to find a CAN driver. Please make sure you have one of the above drivers installed with the library." << std::endl;
		std::cout << "If you want to use a different driver, please add it to the list above." << std::endl;
		return -1;
	}
	isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
	isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Debug); // Change this to Debug to see more information
	isobus::CANHardwareInterface::set_number_of_can_channels(1);
	// isobus::CANHardwareInterface::set_periodic_update_interval(10);
	isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

	if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid()))
	{
		std::cout << "Failed to start CAN hardware interface." << std::endl;
		return -1;
	}

	isobus::NAME ourNAME(0);

	//! Make sure you change these for your device!!!!
	ourNAME.set_arbitrary_address_capable(true);
	ourNAME.set_industry_group(2);
	ourNAME.set_device_class(0);
	ourNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
	ourNAME.set_identity_number(20);
	ourNAME.set_ecu_instance(0);
	ourNAME.set_function_instance(0); // TC #1. If you want to change the TC number, change this.
	ourNAME.set_device_class_instance(0);
	ourNAME.set_manufacturer_code(1407);

	auto serverCF = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(ourNAME, 0, isobus::preferred_addresses::IndustryGroup2::TaskController_MappingComputer); // The preferred address for a TC is defined in ISO 11783
	auto addressClaimedFuture = std::async(std::launch::async, [&serverCF]() {
			while (!serverCF->get_address_valid())
				std::this_thread::sleep_for(std::chrono::milliseconds(100)); });

	// If this fails, probably the update thread is not started
	addressClaimedFuture.wait_for(std::chrono::seconds(5));
	if (!serverCF->get_address_valid())
	{
		std::cout << "Failed to claim address for TC server. The control function might be invalid." << std::endl;
		return -1;
	}

	MyTCServer server(serverCF);
	auto &languageInterface = server.get_language_command_interface();
	languageInterface.set_language_code("en"); // This is the default, but you can change it if you want
	languageInterface.set_country_code("US"); // This is the default, but you can change it if you want
	server.initialize();

	// Initialize speed and distance messages
	isobus::SpeedMessagesInterface speedMessagesInterface(serverCF, false, false, true, false);
	speedMessagesInterface.initialize();

	std::cout << "Task controller server started." << std::endl;

	auto packetHandler = [&server, &speedMessagesInterface](std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data) {
		if (src == 0x7F && pgn == 0xFE) // 254 - Steer Data
		{
			std::uint16_t speed = data[0] | (data[1] << 8);
			std::uint8_t status = data[2];

			// Convert from km/h to mm/s
			speed = speed * 1E5 / 3600;

			speedMessagesInterface.machineSelectedSpeedTransmitData.set_speed_source(isobus::SpeedMessagesInterface::MachineSelectedSpeedData::SpeedSource::NavigationBasedSpeed);
			speedMessagesInterface.machineSelectedSpeedTransmitData.set_machine_direction_of_travel(isobus::SpeedMessagesInterface::MachineDirection::Forward); // TODO: Implement direction
			speedMessagesInterface.machineSelectedSpeedTransmitData.set_machine_speed(speed);
			speedMessagesInterface.machineSelectedSpeedTransmitData.set_machine_distance(0); // TODO: Implement distance

			// TODO: hack to get desired section states. probably want to make a new pgn later when we need more than 16 sections
			std::vector<bool> sectionStates;
			for (std::uint8_t i = 0; i < 8; i++)
			{
				sectionStates.push_back(data[6] & (1 << i));
			}
			for (std::uint8_t i = 0; i < 8; i++)
			{
				sectionStates.push_back(data[7] & (1 << i));
			}

			server.update_section_states(sectionStates);
		}
	};
	udpConnections->set_packet_handler(packetHandler);
	udpConnections->open();

	std::cout << "UDP connections opened." << std::endl;

	std::uint32_t lastHeartbeatTransmit = 0;

	while (running)
	{
		udpConnections->handle_address_detection();
		udpConnections->handle_incoming_packets();

		server.request_measurement_commands();
		if (toggleSectionControl)
		{
			bool currentSectionControlState = false;
			if (!server.get_clients().empty())
			{
				currentSectionControlState = server.get_clients().begin()->second.is_section_control_enabled();
			}
			server.update_section_control_enabled(!currentSectionControlState);
			toggleSectionControl = false;
		}
		server.update();
		speedMessagesInterface.update();

		if (isobus::SystemTiming::time_expired_ms(lastHeartbeatTransmit, 100))
		{
			for (auto &client : server.get_clients())
			{
				auto &state = client.second;
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
				udpConnections->send(0x7F, 0xF0, data);
			}
			lastHeartbeatTransmit = isobus::SystemTiming::get_timestamp_ms();
		}

		// Update again in a little bit
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	inputThread.join();
	server.terminate();
	isobus::CANHardwareInterface::stop();
}
