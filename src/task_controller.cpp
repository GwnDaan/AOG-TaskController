/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "task_controller.hpp"

#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <bitset>
#include <iostream>

void ClientState::set_number_of_sections(std::uint8_t number)
{
	numberOfSections = number;
	sectionSetpointStates.resize(number);
	sectionActualStates.resize(number);
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
	std::cerr << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return 0;
}

void ClientState::set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
{
	ddiToElementNumber[ddi] = elementNumber;
}

MyTCServer::MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction) :
  TaskControllerServer(internalControlFunction,
                       1, // AOG limits to 1 boom
                       16, // AOG limits to 16 sections of unique width
                       16, // 16 channels for position based control
                       isobus::TaskControllerOptions()
                         .with_implement_section_control(), // We support section control
                       TaskControllerVersion::SecondEditionDraft)
{
}

bool MyTCServer::activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &)
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

bool MyTCServer::change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &)
{
	return true;
}

bool MyTCServer::deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF)
{
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &)
{
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

void MyTCServer::identify_task_controller(std::uint8_t)
{
	// When this is called, the TC is supposed to display its TC number for 3 seconds if possible (which is passed into this function).
	// Your TC's number is your function code + 1, in the range of 1-32.
}

void MyTCServer::on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner)
{
	// Cleanup the client state
	clients.erase(partner);
}

void MyTCServer::on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner,
                                             std::uint16_t dataDescriptionIndex,
                                             std::uint16_t elementNumber,
                                             std::uint8_t errorCodesFromClient,
                                             ProcessDataCommands processDataCommand)
{
	// This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
	std::cout << "Received process data acknowledge from client " << int(partner->get_address()) << " for DDI " << dataDescriptionIndex << " element " << elementNumber << " with error codes " << std::bitset<8>(errorCodesFromClient) << " and command " << static_cast<int>(processDataCommand) << std::endl;
}

bool MyTCServer::on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
                                  std::uint16_t dataDescriptionIndex,
                                  std::uint16_t elementNumber,
                                  std::int32_t processDataValue,
                                  std::uint8_t &errorCodes)
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

bool MyTCServer::store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool)
{
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		uploadedPools[partnerCF] = std::queue<std::vector<std::uint8_t>>();
	}
	uploadedPools[partnerCF].push(binaryPool);
	return true;
}

std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &MyTCServer::get_clients()
{
	return clients;
}

void MyTCServer::request_measurement_commands()
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

void MyTCServer::update_section_states(std::vector<bool> &sectionStates)
{
	for (auto &client : clients)
	{
		auto &state = client.second;
		if (!state.is_section_control_enabled())
		{
			// According to standard, the section setpoint states should only be sent when in auto mode
			return;
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
			std::uint8_t ddiOffset = state.get_number_of_sections() / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
			send_section_setpoint_states(client.first, ddiOffset);
		}
	}
}

void MyTCServer::update_section_control_enabled(bool enabled)
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

void MyTCServer::send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
{
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

void MyTCServer::send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
{
	std::cout << "Sending section control state: " << (enabled ? "enabled" : "disabled") << std::endl;
	send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
}
