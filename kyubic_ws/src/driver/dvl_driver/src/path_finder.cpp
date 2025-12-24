/**
 * @file path_finder.cpp
 * @brief Path Finder DVL Interface Library
 * @author R.Ohnishi
 * @date 2024/10/30
 *
 * @details DVL(Path Finder) のデータを取得
 * Reference: YusukeMizoguchi on 2022/05/07.
 ******************************************/

#include "dvl_driver/path_finder.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <thread>

#include "dvl_driver/endian_utils.hpp"
#include "dvl_driver/pd0_types.hpp"

namespace dvl_driver::path_finder
{

// Helper for error reporting
static void report_error(const char * msg)
{
  perror(msg);
  exit(0);
}

Listener::Listener(const char * _address, const int _port, const int _timeout_ms)
: address(_address), port(_port), timeout_ms(_timeout_ms)
{
  // AF_INET: IPv4, SOCK_STREAM: TCP
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    report_error("DVL Listener: ERROR opening socket");
  }

  // Set timeout
  struct timeval tv;
  tv.tv_sec = static_cast<int>(timeout_ms / 1000);
  tv.tv_usec = (timeout_ms - tv.tv_sec * 1000) * 1000;
  std::cout << "Set timeout -> " << tv.tv_sec << "[s] " << tv.tv_usec << "[us]" << std::endl;
  if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
    std::cout << "ERROR setting connect timeout" << std::endl;
  }
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cout << "ERROR setting recv timeout" << std::endl;
  }

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address);
  server.sin_port = htons(port);

  std::cout << "DVL Listener: Connecting to " << address << ":" << port << "..." << std::endl;
  if (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    report_error("DVL Listener: Connect failed");
  } else {
    std::cout << "DVL Listener: Connected." << std::endl;
  }
}

Listener::~Listener()
{
  if (sockfd >= 0) {
    close(sockfd);
  }
}

ssize_t Listener::read(unsigned char * buffer, size_t size)
{
  return ::recv(sockfd, buffer, size, 0);
}

bool Listener::listen()
{
  std::memset(buffer, 0, BUFFER_SIZE);
  ssize_t len = this->read(buffer, BUFFER_SIZE);

  if (len > 0) {
    for (size_t i = 0; i < static_cast<size_t>(len) - 1; ++i) {
      // --- Check for PD0 ---
      if (utils::get_u16(buffer + i) == pd0::DATA_ID) {
        if (parser0.parse(buffer + i, len - i, pd0_data)) {
          last_data_type = 0;
          pd0_data.bottom_track.velocity[1] *= -1;  // y-axis (right plus)
          return true;
        }
      }

      // --- Check for PD5/PD4 ---
      else if (buffer[i] == pd5::DATA_ID) {
        if (parser5.parse(buffer + i, len - i, pd5_data)) {
          last_data_type = 5;
          pd5_data.btm_velocity[1] *= -1;  // y-axis (right plus)
          return true;
        }
      }
    }
  } else if (len == 0) {
    // Connection closed
    std::cerr << "DVL Listener: Connection closed by peer." << std::endl;
  } else {
    // Error or Timeout
    // Perror is noisy on timeout, so maybe suppress unless needed
  }

  return false;
}

bool Listener::has_pd0_data() const { return (last_data_type == 0 && pd0_data.is_valid); }

bool Listener::has_pd5_data() const { return (last_data_type == 5 && pd5_data.is_valid); }

std::shared_ptr<pd0::Pd0Ensemble> Listener::get_pd0_data()
{
  return std::make_shared<pd0::Pd0Ensemble>(pd0_data);
}

std::shared_ptr<pd5::Pd5Ensemble> Listener::get_pd5_data()
{
  return std::make_shared<pd5::Pd5Ensemble>(pd5_data);
}

Sender::Sender(const char * _address, const int _port, const int _timeout_ms)
: address(_address), port(_port), timeout_ms(_timeout_ms)
{
  // AF_INET: IPv4, SOCK_STREAM: TCP
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    report_error("DVL Sender: ERROR opening socket");
  }

  struct timeval tv;
  tv.tv_sec = static_cast<int>(timeout_ms / 1000);
  tv.tv_usec = (timeout_ms - tv.tv_sec * 1000) * 1000;
  std::cout << "Set timeout -> " << tv.tv_sec << "[s] " << tv.tv_usec << "[us]" << std::endl;
  if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
    std::cout << "ERROR setting connect timeout" << std::endl;
  }
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cout << "ERROR setting recv timeout" << std::endl;
  }

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(address);
  server.sin_port = htons(port);

  std::cout << "DVL Sender: Connecting to " << address << ":" << port << "..." << std::endl;
  if (connect(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0) {
    report_error("DVL Sender: Connect failed");
  } else {
    std::cout << "DVL Sender: Connected." << std::endl;
  }
}

Sender::~Sender()
{
  if (sockfd >= 0) {
    close(sockfd);
  }
}

ssize_t Sender::read(unsigned char * buffer, size_t size)
{
  return ::recv(sockfd, buffer, size, 0);
}

void Sender::flush_buffer()
{
  char temp_buf[1024];
  int ret;

  // SAFEGUARD: Limit the number of iterations to prevent infinite loops
  // in case the sender is flooding the socket faster than we can read.
  const int MAX_ITERATIONS = 1000;
  int loop_count = 0;

  while (loop_count < MAX_ITERATIONS) {
    // Try to read data in non-blocking mode (MSG_DONTWAIT).
    ret = recv(sockfd, temp_buf, sizeof(temp_buf), MSG_DONTWAIT);

    if (ret < 0) {
      // Check if the buffer is empty
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      // Handle actual errors
      perror("recv failed");
      break;
    }

    if (ret == 0) {
      // The peer has performed an orderly shutdown (connection closed).
      break;
    }
    loop_count++;
  }
}

bool Sender::send_cmd(const std::string & _cmd, const uint & wait_time, bool newline)
{
  std::string cmd(_cmd);
  if (newline) cmd += CRCF;

  if (send(sockfd, cmd.c_str(), cmd.size(), 0) != (ssize_t)cmd.size()) {
    return false;
  }
  if (wait_time > 0) {
    std::this_thread::sleep_for(std::chrono::seconds(wait_time));
  }
  return true;
}

bool Sender::send_break_cmd() { return send_cmd(break_cmd, 4); }

bool Sender::send_ping_cmd() { return send_cmd(ping_cmd); }

}  // namespace dvl_driver::path_finder
