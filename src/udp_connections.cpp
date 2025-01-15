/**
 * @author Daan Steenbergen
 * @brief UDP connections to communicate with AgOpenGPS
 * @version 0.1
 * @date 2025-1-14
 *
 * @copyright 2025 Daan Steenbergen
 */

#include "udp_connections.hpp"
#include <cassert>
#include <iostream>

UdpConnections::UdpConnections(std::shared_ptr<Settings> settings, boost::asio::io_context &ioContext) :
  settings(settings),
  udpConnection(ioContext),
  udpConnectionAddressDetection(ioContext)
{
	assert(settings && "Settings must not be null");
}
void UdpConnections::set_packet_handler(PacketCallback packetCallback)
{
	this->packetCallback = packetCallback;
}

bool UdpConnections::open()
{
	// Set up the UDP server
	udpConnection.open(udp::v4());
	udpConnection.set_option(boost::asio::socket_base::broadcast(true));
	udpConnection.non_blocking(true);
	udpConnection.bind(get_local_endpoint());

	// Set up another UDP server for Address Detection
	udpConnectionAddressDetection.open(udp::v4());
	udpConnectionAddressDetection.set_option(boost::asio::socket_base::broadcast(true));
	udpConnectionAddressDetection.non_blocking(true);
	udp::endpoint local_any_endpoint(boost::asio::ip::address_v4::any(), 8888); // Bind to 0.0.0.0 to receive packets on all interfaces
	udpConnectionAddressDetection.bind(local_any_endpoint);

	return true;
}

void UdpConnections::close()
{
	udpConnection.close();
	udpConnectionAddressDetection.close();
}

udp::endpoint UdpConnections::get_local_endpoint() const
{
	auto subnet = settings->get_subnet();
	try
	{
		boost::asio::io_context io_context;
		boost::asio::ip::tcp::resolver resolver(io_context);
		auto endpoints = resolver.resolve(boost::asio::ip::host_name(), "");

		std::cout << "Available IP addresses:" << std::endl;
		for (const auto &endpoint : endpoints)
		{
			const auto &addr = endpoint.endpoint().address();
			std::cout << "- " << addr.to_string() << std::endl;

			if (addr.is_v4())
			{
				auto octets = addr.to_v4().to_bytes();
				if (std::equal(subnet.begin(), subnet.begin() + 3, octets.begin()))
				{
					std::cout << "Found local endpoint address " << addr.to_string() << ", which matches the subnet " << settings->get_subnet_string() << std::endl;
					return udp::endpoint(addr, 8888);
				}
			}
		}
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
	}

	std::cout << "No suitable IP address found that matches the subnet " << settings->get_subnet_string() << ", using loopback address.\n";
	return udp::endpoint(boost::asio::ip::address_v4::loopback(), 8888);
}

std::uint8_t UdpConnections::calculate_crc(std::span<std::uint8_t> data)
{
	int result = 0;
	for (uint8_t i = 0; i < data.size(); i++)
	{
		result += data[i];
	}
	return static_cast<std::uint8_t>(result & 0xFF);
}

void UdpConnections::handle_incoming_packets()
{
	static std::array<std::uint8_t, 512> rxBuffer;
	static std::size_t rxIndex = 0;

	// Peek to see if we have any data
	boost::system::error_code error_code;
	udp::endpoint sender_endpoint;
	size_t bytesReceived = udpConnection.receive_from(boost::asio::buffer(rxBuffer.data() + rxIndex, sizeof(rxBuffer) - rxIndex), sender_endpoint, 0, error_code);

	if (error_code == boost::asio::error::would_block)
	{
		// No data available
	}
	else if (!error_code)
	{
		rxIndex += bytesReceived;
		std::uint8_t index = 0;

		while (rxIndex >= 8)
		{
			std::uint16_t start = (rxBuffer[index++] << 8) | rxBuffer[index++];
			if (start == PACKET_START)
			{
				std::uint8_t src = rxBuffer[index++];
				std::uint8_t pgn = rxBuffer[index++];
				std::uint8_t len = rxBuffer[index++];
				std::uint8_t crc = rxBuffer[index + len];

				// Check CRC (skip start of packet, but include source, PGN, length and data)
				// std::uint8_t crcCalc = calculate_crc({ rxBuffer.data() + index - 3, static_cast<size_t>(len) + 3 });
				// if (crc != crcCalc)
				// {
				//  std::cerr << "CRC mismatch (for PGN " << static_cast<int>(pgn) << "?), expected " << static_cast<int>(crc) << " but got " << static_cast<int>(crcCalc) << std::endl;
				// 	rxIndex = 0;
				// 	break;
				// }

				if (packetCallback)
				{
					packetCallback(src, pgn, { rxBuffer.data() + index, len });
				}
				index += len + 1;
			}
			else
			{
				// Unknown start of message, reset buffer
				std::cout << "Unknown start of message: 0x" << std::hex << start << std::endl;
				rxIndex = 0;
			}

			// Move any remaining data to the front of the buffer
			if (index < rxIndex)
			{
				std::memmove(rxBuffer.data(), rxBuffer.data() + index, rxIndex - index);
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
		std::cerr << "Error while receiving data: " << error_code.message() << std::endl;
	}
}

void UdpConnections::handle_address_detection()
{
	static std::array<std::uint8_t, 512> rxBuffer;
	static std::size_t rxIndex = 0;

	// Peek to see if we have any data
	boost::system::error_code error_code;
	udp::endpoint sender_endpoint;
	size_t bytesReceived = udpConnectionAddressDetection.receive_from(boost::asio::buffer(rxBuffer.data() + rxIndex, sizeof(rxBuffer) - rxIndex), sender_endpoint, 0, error_code);

	if (error_code == boost::asio::error::would_block)
	{
		// No data available
	}
	else if (!error_code)
	{
		rxIndex += bytesReceived;
		std::uint8_t index = 0;

		while (rxIndex >= 8)
		{
			std::uint16_t start = (rxBuffer[index++] << 8) | rxBuffer[index++];
			if (start == PACKET_START)
			{
				std::uint8_t src = rxBuffer[index++];
				std::uint8_t pgn = rxBuffer[index++];
				std::uint8_t len = rxBuffer[index++];
				std::uint8_t crc = rxBuffer[index + len];

				// Check CRC (skip start of packet, but include source, PGN, length and data)
				// std::uint8_t crcCalc = calculate_crc({ rxBuffer.data() + index - 3, static_cast<size_t>(len) + 3 });
				// if (crc != crcCalc)
				// {
				// 	std::cerr << "CRC mismatch (for PGN " << static_cast<int>(pgn) << "?), expected " << static_cast<int>(crc) << " but got " << static_cast<int>(crcCalc) << std::endl;
				// 	rxIndex = 0;
				// 	break;
				// }

				if (src == 0x7F && pgn == 0xC9 && len == 5 // 127 is source, AGIO, 201 is subnet
				    && rxBuffer[index++] == 0xC9 && rxBuffer[index++] == 0xC9)
				{
					// 7-8-9 is IP0,IP1,IP2
					settings->set_subnet({ rxBuffer[index++], rxBuffer[index++], rxBuffer[index++] });

					std::cout << "Subnet from AOG: ";
					std::cout << int(settings->get_subnet()[0]) << ".";
					std::cout << int(settings->get_subnet()[1]) << ".";
					std::cout << int(settings->get_subnet()[2]);
					std::cout << " rebinding UPD connection " << std::endl;
					udpConnection.close();
					udpConnection.open(udp::v4());
					udpConnection.set_option(boost::asio::socket_base::broadcast(true));
					udpConnection.non_blocking(true);
					udpConnection.bind(get_local_endpoint());

					index += len - 4;
				}
				else
				{
					index += len + 1;
				}
			}
			else
			{
				// Unknown start of message, reset buffer
				std::cout << "Unknown start of message: 0x" << std::hex << start << std::endl;
				rxIndex = 0;
			}

			// Move any remaining data to the front of the buffer
			if (index < rxIndex)
			{
				std::memmove(rxBuffer.data(), rxBuffer.data() + index, rxIndex - index);
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
		std::cerr << "Error while receiving data: " << error_code.message() << std::endl;
	}
}

bool UdpConnections::send(std::uint8_t src, std::uint8_t pgn, std::span<std::uint8_t> data)
{
	std::array<std::uint8_t, 512> txBuffer;
	std::size_t index = 0;

	txBuffer[index++] = PACKET_START >> 8;
	txBuffer[index++] = PACKET_START & 0xFF;
	txBuffer[index++] = src;
	txBuffer[index++] = pgn;
	txBuffer[index++] = data.size();
	std::copy(data.begin(), data.end(), txBuffer.begin() + index);
	index += data.size();

	txBuffer[index] = calculate_crc({ txBuffer.data() + 2, index - 2 });

	auto subnet = settings->get_subnet();
	boost::asio::ip::address_v4 broadcast_address = boost::asio::ip::make_address_v4(std::to_string(subnet[0]) + "." +
	                                                                                 std::to_string(subnet[1]) + "." +
	                                                                                 std::to_string(subnet[2]) + ".255");

	udp::endpoint broadcast_endpoint(broadcast_address, 9999);
	udpConnection.send_to(boost::asio::buffer(txBuffer, index + 1), broadcast_endpoint);

	return true;
}
