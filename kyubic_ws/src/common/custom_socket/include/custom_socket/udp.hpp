/**
 * @file udp.hpp
 * @brief UDP Socket Communication Wrapper
 * @author R.Ohnishi
 * @date 2026/01/19
 *
 * @details UDP通信を簡単に扱えるようにしたライブラリ
 */

#ifndef _UDP_HPP
#define _UDP_HPP

#include <netinet/in.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace common
{

/**
 * @brief UDP payload and its source endpoint.
 */
struct UdpDatagram
{
  std::vector<uint8_t> data;   ///< Received UDP payload.
  std::string sender_address;  ///< Sender IPv4 address.
  uint16_t sender_port;        ///< Sender UDP port.
};

/**
 * @class UdpSocket
 * @brief UDP (Sender/Listener) Class
 */
class UdpSocket
{
public:
  /**
    * @brief Creates a socket and configures basic options.
    */
  UdpSocket();

  /**
    * @brief Closes the socket.
    */
  ~UdpSocket();

  // --- Configuration ---

  /**
    * @brief Sets the send/receive timeout.
    * @param sec Timeout duration in seconds.
    */
  void setTimeout(int sec);

  /**
    * @brief Sets the send/receive timeout with millisecond precision.
    * @param timeout Timeout duration. It must not be negative.
    */
  void setTimeout(std::chrono::milliseconds timeout);

  /**
    * @brief Enables or disables broadcast.
    * @param enable Set to true to enable broadcast.
    */
  void setBroadcast(bool enable);

  // --- Listener Methods ---

  /**
    * @brief Binds to a specified port and prepares for receiving data.
    * @param port Port number to listen on.
    * @return true on success, false on failure.
    */
  bool bind(int port);

  /**
    * @brief Binds to an IPv4 address and port.
    * @param address IPv4 address to bind. Use ``0.0.0.0`` for all interfaces.
    * @param port Port number to listen on.
    * @return true on success, false on failure.
    */
  bool bind(const std::string & address, int port);

  /**
    * @brief Receives data.
    * @param max_len Maximum buffer size for reception.
    * @return std::vector<uint8_t> Received data (returns an empty vector on timeout or error).
    */
  std::vector<uint8_t> receive(size_t max_len = 1024);

  /**
    * @brief Receives data together with its source endpoint.
    * @param max_len Maximum buffer size for reception.
    * @return Received datagram, or ``std::nullopt`` after a receive timeout.
    * @throws std::runtime_error When reception fails for a reason other than timeout.
    */
  std::optional<UdpDatagram> receiveFrom(size_t max_len = 1024);

  // --- Sender Methods ---

  /**
    * @brief Sets a fixed destination address (uses `connect`).
    * This enables the use of the send() method.
    * @param host Destination IP address.
    * @param port Destination port number.
    * @return true on success.
    */
  bool setDestination(const std::string & host, int port);

  /**
    * @brief Sends data to the fixed destination.
    * @param data Data to be sent.
    * @return ssize_t Number of bytes sent.
    */
  ssize_t send(const std::vector<uint8_t> & data);

  /**
    * @brief Sends data to a specified destination (uses `sendto`).
    * Can be used without calling setDestination().
    * @param data Data to be sent.
    * @param host Destination IP address.
    * @param port Destination port number.
    * @return ssize_t Number of bytes sent.
    */
  ssize_t sendTo(const std::vector<uint8_t> & data, const std::string & host, int port);

private:
  int sockfd_;
  struct sockaddr_in remote_addr_;  ///< Destination address set by setDestination
  bool is_connected_;               ///< Connection status flag
};

}  // namespace common

#endif  // _UDP_SOCKET_HPP
