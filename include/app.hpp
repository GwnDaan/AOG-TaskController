/**
 * @author Daan Steenbergen
 * @brief The main application class
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#pragma once

#include <boost/asio.hpp>
#include <memory>

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/isobus_speed_distance_messages.hpp"
#include "isobus/isobus/nmea2000_message_interface.hpp"

#include "settings.hpp"
#include "task_controller.hpp"
#include "udp_connections.hpp"

class Application
{
public:
	Application(std::shared_ptr<isobus::CANHardwarePlugin> canDriver);

	bool initialize();
	bool update();
	void stop();

private:
	std::shared_ptr<Settings> settings = std::make_shared<Settings>();
	boost::asio::io_context ioContext = boost::asio::io_context();
	std::shared_ptr<UdpConnections> udpConnections = std::make_shared<UdpConnections>(settings, ioContext);

	std::shared_ptr<isobus::CANHardwarePlugin> canDriver;
	std::shared_ptr<MyTCServer> tcServer;
	std::unique_ptr<isobus::SpeedMessagesInterface> speedMessagesInterface;
	std::unique_ptr<isobus::NMEA2000MessageInterface> nmea2000MessageInterface;
	std::uint8_t nmea2000SequenceIdentifier = 0;
};
