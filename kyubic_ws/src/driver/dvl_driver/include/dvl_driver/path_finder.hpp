/**
 * @file path_finder.hpp
 * @brief Path Finder DVL Interface Library
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得
 * Reference: YusukeMizoguchi on 2022/05/07.
 ******************************************/

#ifndef _PATH_FINDER_HPP
#define _PATH_FINDER_HPP

#include <netinet/in.h>

#include <memory>
#include <string>

#include "dvl_driver/pd0_parser.hpp"
#include "dvl_driver/pd0_types.hpp"
#include "dvl_driver/pd5_parser.hpp"
#include "dvl_driver/pd5_types.hpp"

/**
 * @namespace driver::dvl_driver::path_finder
 * @brief For path_finder
 */
namespace driver::dvl_driver::path_finder
{

/**
 * @class Listener
 * @brief Handles TCP reception of DVL data and dispatches parsing to PD0 or PD5 parsers.
 */
class Listener
{
private:
  const char * address;
  const int port;
  const int timeout_ms;

  int sockfd;
  struct sockaddr_in server;

  static const size_t BUFFER_SIZE = 65536;
  unsigned char buffer[BUFFER_SIZE];

  // Parsers
  pd0::Pd0Parser parser0;
  pd5::Pd5Parser parser5;

  // Data Storage
  pd0::Pd0Ensemble pd0_data;
  pd5::Pd5Ensemble pd5_data;

  // 0 = PD0, 5 = PD5, -1 = None
  int last_data_type = -1;

public:
  /**
   * @brief Constructor for Listener
   * @param _address Ip address of Path Finder (e.g., "192.168.1.100")
   * @param _port TCP port to connect to (e.g., 1034 for data)
   * @param _timeout_ms Socket timeout in milliseconds
   * @details Socket creation with IPv4 and TCP. Socket connection based on address and port
   */
  explicit Listener(const char * _address, const int _port, const int _timeout_ms = 1000);

  /**
   * @brief Destructor. Closes the socket.
   */
  ~Listener();

  /**
   * @brief Listen for incoming data and attempt to parse it.
   * @details Reads data from the socket and checks headers to determine if it is PD0 or PD5.
   * @return true if a valid packet (PD0 or PD5) was received and parsed.
   */
  bool listen();

  /**
   * @brief Read raw bytes from the socket.
   * @param buffer Destination buffer
   * @param size Maximum size to read
   * @return Number of bytes read, or -1 on error
   */
  ssize_t read(unsigned char * buffer, size_t size);

  /**
   * @brief Check if valid PD0 data is currently available.
   * @return true if the last successful parse was PD0.
   */
  bool has_pd0_data() const;

  /**
   * @brief Check if valid PD5 data is currently available.
   * @return true if the last successful parse was PD5 (or PD4).
   */
  bool has_pd5_data() const;

  /**
   * @brief Retrieve the latest parsed PD0 data.
   * @return Shared pointer to a copy of the PD0 data structure.
   */
  std::shared_ptr<pd0::Pd0Ensemble> get_pd0_data();

  /**
   * @brief Retrieve the latest parsed PD5 data.
   * @return Shared pointer to a copy of the PD5 data structure.
   */
  std::shared_ptr<pd5::Pd5Ensemble> get_pd5_data();
};

/**
 * @class Sender
 * @brief Handles TCP transmission of commands to the DVL.
 */
class Sender
{
  const char * address;
  const int port;
  const int timeout_ms;

  int sockfd;
  struct sockaddr_in server;

  const std::string CRCF = "\r\n";
  const std::string ping_cmd = "CS";
  const std::string break_cmd = "===";

public:
  /**
   * @brief Constructor for Sender
   * @param _address IP address of the DVL
   * @param _port TCP port to connect to (usually 1033 for commands)
   * @param _timeout_ms Socket timeout in milliseconds
   */
  explicit Sender(const char * _address, const int _port, const int _timeout_ms = 1000);

  /**
   * @brief Destructor. Closes the socket.
   */
  ~Sender();

  /**
   * @brief Read response from DVL after sending a command.
   * @param buffer Destination buffer
   * @param size Maximum size to read
   * @return Number of bytes read
   */
  ssize_t read(unsigned char * buffer, size_t size);

  /**
   * @brief Clear socket buffer.
   */
  void flush_buffer();

  /**
   * @brief Send a generic string command to the DVL.
   * @param cmd Command string (e.g., "CR1\r\n")
   * @param wait_time Time to wait after sending (seconds)
   * @return true if sent successfully
   */
  bool send_cmd(const std::string & cmd, const uint & wait_time = 0, bool newline = true);

  /**
   * @brief Send a Soft Break ("===") to wake up the DVL.
   * @return true if sent successfully
   */
  bool send_break_cmd();

  /**
   * @brief Send the Start Pinging command ("CS").
   * @return true if sent successfully
   */
  bool send_ping_cmd();
};

}  // namespace driver::dvl_driver::path_finder

#endif  // _PATH_FINDER_HPP
