/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2024-12-16
 *
 * @copyright 2024 Daan Steenbergen
 *
 */

#include <boost/asio.hpp>
#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <future>

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/hardware_integration/available_can_drivers.hpp"
#include "isobus/isobus/isobus_preferred_addresses.hpp"

#include "console_logger.cpp"
#include <bitset>
#include <map>

constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;

enum SectionState : std::uint8_t
{
    OFF = 0,          ///< Section is off
    ON = 1,           ///< Section is on
    ERROR_SATE = 2,   ///< Section is in an error state
    NOT_INSTALLED = 3 ///< Section is not installed
};

using boost::asio::ip::udp;

boost::asio::io_service io_service;
udp::socket udpConnection(io_service);
boost::asio::io_service io_serviceAD;
udp::socket udpConnectionAD(io_serviceAD);
static std::shared_ptr<isobus::ControlFunction> clientTC = nullptr;

//TODO load these from settings file...
uint8_t MDL_IP1 = 192;
uint8_t MDL_IP2 = 168;
uint8_t MDL_IP3 = 1; 


static std::atomic_bool running = {true};
void signal_handler(int)
{
    running = false;
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

private:
    std::uint8_t numberOfSections;
    std::vector<std::uint8_t> sectionSetpointStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
    std::vector<std::uint8_t> sectionActualStates;   // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
};

// Create the task controller server object, this will handle all the ISOBUS communication for us
class MyTCServer : public isobus::TaskControllerServer
{
public:
    MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction) : TaskControllerServer(internalControlFunction,
                                                                                                                1,  // AOG limits to 1 boom
                                                                                                                64, // AOG limits to 64 sections
                                                                                                                16, // 16 channels for position based control
                                                                                                                isobus::TaskControllerOptions()
                                                                                                                    .with_implement_section_control())
    {
    }

    bool activate_object_pool(std::shared_ptr<isobus::ControlFunction> client, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &) override
    {
        clientTC = client;
        return true;
    }

    bool change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &) override
    {
        return true;
    }

    bool deactivate_object_pool(std::shared_ptr<isobus::ControlFunction>) override
    {
        return true;
    }

    bool delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction>, ObjectPoolDeletionErrors &) override
    {
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

    void on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, std::uint16_t, std::uint8_t, ProcessDataCommands) override
    {
        // This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
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

            // AOG uses 1 bit per section, i.e. 0 = off, 1 = on
            // std::uint8_t numberOfBytes = clients[partner].get_number_of_sections() / 8;
            // if (clients[partner].get_number_of_sections() % 8 > 0)
            // {
            //     // Add an extra byte if there are any remaining sections that don't fit in a multiple of 8
            //     numberOfBytes++;
            // }
            std::vector<uint8_t> AOG = {0x80, 0x81, 0x70, 0x80, clients[partner].get_number_of_sections()}; // 0x700x80 = TC -> AOG, 0x700x00 = AOG -> TC

            std::uint8_t sectionIndex = 0;

            while (sectionIndex < clients[partner].get_number_of_sections())
            {
                if (clients[partner].get_section_actual_state(sectionIndex) == SectionState::ON)
                {
                    AOG.push_back(1);
                }
                else
                {
                    AOG.push_back(0);
                }
                //AOG.push_back(0); // filler-byte, for the moment, not used. 2 bytes per section currently
                sectionIndex++;
                std::cout << std::endl;
            }
            // add the checksum
            uint8_t CK_A = 0;
            for (uint8_t i = 2; i < AOG.size(); i++) // shouldn't be -1, as we have all data bytes at this point
            {
                CK_A = (CK_A + AOG[i]);
            }
            AOG.push_back(CK_A) ; // & 0xFF);

            // for (const auto &byte : AOG)
            // {
            //     std::cout << std::dec << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::endl;

            //udp::endpoint broadcast_endpoint(boost::asio::ip::address_v4::broadcast(), 9999);
            boost::asio::ip::address_v4 listen_address = boost::asio::ip::address_v4::from_string("192.168.5.255"); // wpon't work on 255.255.255.255, need to pick correct subnet
            // better to follow the AOG example of scanning/setting subnets perhaps?
            udp::endpoint broadcast_endpoint(listen_address, 9999); // AOG listens on 9999, not 8888
            udpConnection.send_to(boost::asio::buffer(AOG, sizeof(AOG)), broadcast_endpoint);

        }
        break;
        }

        return true;
    }

    bool store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool) override
    {
        // Initialize for the partner control function
        clients[partnerCF] = ClientState();

        isobus::DeviceDescriptorObjectPool pool;
        pool.set_task_controller_compatibility_level(3);
        if (pool.deserialize_binary_object_pool(binaryPool.data(), static_cast<std::uint32_t>(binaryPool.size()), partnerCF->get_NAME()))
        {
            std::cout << "Successfully deserialized device descriptor object pool." << std::endl;
            auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(pool);
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
            clients[partnerCF].set_number_of_sections(numberOfSections);
        }
        else
        {
            std::cout << "Failed to deserialize device descriptor object pool." << std::endl;
            return false;
        }
        return true;
    }

    std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> &get_clients()
    {
        return clients;
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

    static udp::endpoint get_local_endpoint()
    {
      try {
        boost::asio::io_context io_context;

        // Retrieve a list of all network interfaces
        boost::asio::ip::tcp::resolver resolver(io_context);
        boost::asio::ip::tcp::resolver::query query(boost::asio::ip::host_name(), "");
        boost::asio::ip::tcp::resolver::iterator endpoints = resolver.resolve(query);

        std::cout << "Available IP addresses:" << std::endl;
        for (auto it = endpoints; it != boost::asio::ip::tcp::resolver::iterator(); ++it) {
            boost::asio::ip::address addr = it->endpoint().address();
            std::cout << "- " << addr.to_string() << std::endl;
            if(addr.is_v4()){
                std::string ip_str = addr.to_string();

                // Split the string into parts and check the prefix
                std::istringstream ip_stream(ip_str);
                std::string part;
                std::vector<int> octets;

                while (std::getline(ip_stream, part, '.')) {
                    octets.push_back(std::stoi(part));
                }

                if (octets.size() == 4 && 
                    octets[0] == MDL_IP1 && 
                    octets[1] == MDL_IP2 &&
                    octets[2] == MDL_IP3 ) {
                    std::cout << "Local endpoint address: " << addr.to_string() << std::endl;
                    return udp::endpoint(addr, 8888);
                }
            }
        }
      } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
      }
      return udp::endpoint(
            boost::asio::ip::address_v4::from_string(
                (std::ostringstream() << static_cast<int>(MDL_IP1) << "."
                    << static_cast<int>(MDL_IP2) << "."
                    << static_cast<int>(MDL_IP3) << "."
                    << "0").str()),
            8888); 

    }

private:
    void send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset)
    {
        std::uint8_t sectionOffset = ddiOffset * 16;
        std::uint32_t value = 0;
        for (std::uint8_t i = 0; i < 16; i++)
        {
            value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
        }

        std::cout << "Sending setpoint states for DDI offset " << static_cast<int>(ddiOffset) << " with states: ";
        for (std::uint8_t i = 0; i < 16; i++)
        {
            std::cout << static_cast<int>(clients[client].get_section_setpoint_state(sectionOffset + i)) << " ";
        }
        std::cout << std::endl;

        send_set_value_and_acknowledge(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset, 2, value);
    }


    std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> clients;
};

int main()
{
    std::signal(SIGINT, signal_handler);

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
    auto addressClaimedFuture = std::async(std::launch::async, [&serverCF]()
                                           {
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
    languageInterface.set_country_code("US");  // This is the default, but you can change it if you want
    server.initialize();

    std::cout << "Task controller server started." << std::endl;

    // Set up the UDP server
    udpConnection.open(udp::v4());
    udpConnection.set_option(boost::asio::socket_base::broadcast(true));
    udpConnection.non_blocking(true);

    udpConnection.bind(MyTCServer::get_local_endpoint());

//    std::cout << "Local endpoint address: " << get_local_endpoint().address().to_string() << std::endl;
    std::cout << "Broadcast address: " << boost::asio::ip::address_v4::broadcast().to_string() << std::endl;

    // Set up anoter UDP server for Address Detection
    udpConnectionAD.open(udp::v4());
    udpConnectionAD.set_option(boost::asio::socket_base::broadcast(true));
    udpConnectionAD.non_blocking(true);
    udp::endpoint local_any_endpoint(boost::asio::ip::address_v4::any(), 8888); // Module Address bind is sent to here
    std::cout << "udpConnectionAD " << std::endl;
    udpConnectionAD.bind(local_any_endpoint);

    std:cout << "udpConnectionAD address: " << local_any_endpoint.address().to_string() << std::endl;

    uint8_t rxBuffer[512];
    size_t rxIndex = 0;
    uint8_t rxADBuffer[512];
    size_t rxADIndex = 0;
    while (running)
    {
        // Peek to see if we have any data
        boost::system::error_code ec;
        udp::endpoint sender_endpoint;
        // AWRECEIVE
        size_t bytesReceived = udpConnection.receive_from(boost::asio::buffer(rxBuffer + rxIndex, sizeof(rxBuffer) - rxIndex),  sender_endpoint, 0, ec);

        if (ec == boost::asio::error::would_block)
        {
            // No data available
        }
        else if (!ec)
        {
            rxIndex += bytesReceived;
            std::uint8_t index = 0;

            while (rxIndex >= 8)
            {
                if (rxBuffer[index++] == 0x80 && rxBuffer[index++] == 0x81)
                {
                    std::uint8_t src = rxBuffer[index++];
                    std::uint8_t pgn = rxBuffer[index++];
                    std::uint8_t len = rxBuffer[index++];
                    if (src == 0x70 && pgn == 0x00) // 239 - EF Machine Data
                    {
                        // We only care about the sections (index + 6)
                        std::vector<bool> sectionStates;
                        for (std::uint8_t i = 0; i < len; i++)
                        {
                            sectionStates.push_back(rxBuffer[index++] == 1); // no offset now, new PGN
                        }
                        server.update_section_states(sectionStates);
                        //index += len + 1; // what happens here if >8 sections?
                    }
                    else if (src == 0x70) {
                        //std::cout << "Saw my own ISOBUS traffic" << std::endl;
                    }
                    else {
                        // Unknown PGN, reset buffer
                        //std::cout << "Unknown PGN: " << std::hex << static_cast<int>(pgn) << std::endl;
                        rxIndex = 0;
                    }
                }
                else
                {
                    // Unknown start of message, reset buffer
                    //std::cout << "Unknown start of message: " << std::hex << static_cast<int>(rxBuffer[index - 2]) << " " << static_cast<int>(rxBuffer[index - 1]) << std::endl;
                    rxIndex = 0;
                }

                // Move any remaining data to the front of the buffer
                if (index < rxIndex)
                {
                    std::memmove(rxBuffer, rxBuffer + index, rxIndex - index);
                    rxIndex -= index;
                }
                else
                {
                    rxIndex = 0;
                }
            }
        }
        else
        {
            // Handle other errors
            isobus::CANStackLogger::error("UDP receive error: " + ec.message());
        }

        server.update();

        // Update again in a little bit
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        //Check for AOG subnet
        // AWRECEIVE
        bytesReceived = udpConnectionAD.receive_from(boost::asio::buffer(rxADBuffer + rxADIndex, sizeof(rxADBuffer) - rxADIndex),  sender_endpoint, 0, ec);

        if (ec == boost::asio::error::would_block)
        {
            // No data available
        }
        else if (!ec)
        {

//            std::cout << "Check for AOG subnet" << std::endl;
            std::uint8_t index = 0;

            rxADIndex += bytesReceived;
            while (rxADIndex >= 8)
            {
                if (rxADBuffer[index++] == 0x80 && rxADBuffer[index++] == 0x81) //128, 129
                {
                    index = 2;
                    std::uint8_t src = rxADBuffer[index++];
                    std::uint8_t pgn = rxADBuffer[index++];
                    if (src == 0x7F && pgn == 0xC9 && rxADBuffer[index++] == 0x05  // 127 is source, AGIO, 201 is subnet 
                     && rxADBuffer[index++] == 0xC9 && rxADBuffer[index++] == 0xC9 ) 
                    {
                        //7-8-9 is IP0,IP1,IP2
                        MDL_IP1 = static_cast<int>(rxADBuffer[index++]);
                        MDL_IP2 = static_cast<int>(rxADBuffer[index++]);
                        MDL_IP3 = static_cast<int>(rxADBuffer[index++]);

                        std::cout << "Subnet from AOG: ";
                        std::cout << static_cast<int>(MDL_IP1) << ".";
                        std::cout << static_cast<int>(MDL_IP2) << ".";
                        std::cout << static_cast<int>(MDL_IP3);
                        std::cout << " rebinding UPD connection " << std::endl;
                        udpConnection.close();
                        udpConnection.open(udp::v4());
                        udpConnection.set_option(boost::asio::socket_base::broadcast(true));
                        udpConnection.non_blocking(true);
                        udpConnection.bind(MyTCServer::get_local_endpoint());
                    }
                    else {
                        // Unknown PGN, reset buffer
//                        std::cout << "Unknown PGN: " << std::hex << static_cast<int>(pgn) << std::endl;
                        rxADIndex = 0;
                    }
                }
                else
                {
                    // Unknown start of message, reset buffer
                    //std::cout << "Unknown start of message: " << std::hex << static_cast<int>(rxADBuffer[index - 2]) << " " << static_cast<int>(rxADBuffer[index - 1]) << std::endl;
                    rxADIndex = 0;
                }

                // Move any remaining data to the front of the buffer
                if (index < rxADIndex)
                {  
                    std::memmove(rxADBuffer, rxADBuffer + index, rxADIndex - index);
                    rxADIndex -= index;
                }
                else
                {
                    rxADIndex = 0;
                }
            }
        }
        else
        {
            // Handle other errors
            isobus::CANStackLogger::error("UDP receive error: " + ec.message());
        }
    }

    server.terminate();
    isobus::CANHardwareInterface::stop();
}