/**
 * @file udp.cpp
 * @brief UDP Socket Communication Wrapper
 * @author R.Ohnishi
 * @date 2026/01/19
 *
 * @details UDP通信を簡単に扱えるようにしたライブラリ
 */

#include "custom_socket/udp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace common
{

UdpSocket::UdpSocket() : sockfd_(-1), is_connected_(false)
{
  // Create socket
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    throw std::runtime_error("Socket creation failed");
  }

  // Enable reused address
  int yes = 1;
  setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
}

UdpSocket::~UdpSocket()
{
  if (sockfd_ >= 0) {
    close(sockfd_);
  }
}

void UdpSocket::setTimeout(int sec) { setTimeout(std::chrono::seconds(sec)); }

void UdpSocket::setTimeout(std::chrono::milliseconds timeout)
{
  if (timeout.count() < 0) {
    throw std::invalid_argument("UDP timeout must not be negative");
  }
  struct timeval tv;
  tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

void UdpSocket::setBroadcast(bool enable)
{
  int yes = enable ? 1 : 0;
  setsockopt(sockfd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
}

bool UdpSocket::bind(int port) { return bind("0.0.0.0", port); }

bool UdpSocket::bind(const std::string & address, int port)
{
  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
    std::cerr << "Invalid bind address: " << address << std::endl;
    return false;
  }

  if (::bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Bind failed");
    return false;
  }
  return true;
}

std::vector<uint8_t> UdpSocket::receive(size_t max_len)
{
  try {
    auto datagram = receiveFrom(max_len);
    return datagram ? std::move(datagram->data) : std::vector<uint8_t>{};
  } catch (const std::runtime_error & error) {
    std::cerr << error.what() << std::endl;
    return {};
  }
}

std::optional<UdpDatagram> UdpSocket::receiveFrom(size_t max_len)
{
  if (max_len == 0U) {
    throw std::invalid_argument("UDP receive buffer must not be empty");
  }
  std::vector<uint8_t> buffer(max_len);
  struct sockaddr_in sender_addr;
  socklen_t addr_len = sizeof(sender_addr);

  // Receive via recvfrom (receiving possible regardless of connection state)
  ssize_t len =
    ::recvfrom(sockfd_, buffer.data(), max_len, 0, (struct sockaddr *)&sender_addr, &addr_len);

  // Timeout or Error
  if (len < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::nullopt;
    }
    throw std::runtime_error(std::string("recvfrom failed: ") + std::strerror(errno));
  }

  buffer.resize(len);
  char sender_address[INET_ADDRSTRLEN]{};
  if (
    inet_ntop(AF_INET, &sender_addr.sin_addr, sender_address, sizeof(sender_address)) == nullptr) {
    throw std::runtime_error(std::string("inet_ntop failed: ") + std::strerror(errno));
  }
  return UdpDatagram{std::move(buffer), sender_address, ntohs(sender_addr.sin_port)};
}

bool UdpSocket::setDestination(const std::string & host, int port)
{
  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port = htons(port);

  if (inet_pton(AF_INET, host.c_str(), &remote_addr_.sin_addr) <= 0) {
    std::cerr << "Invalid address: " << host << std::endl;
    return false;
  }

  // UDP connect
  if (::connect(sockfd_, (struct sockaddr *)&remote_addr_, sizeof(remote_addr_)) < 0) {
    perror("Connect failed");
    return false;
  }

  is_connected_ = true;
  return true;
}

ssize_t UdpSocket::send(const std::vector<uint8_t> & data)
{
  if (!is_connected_) {
    std::cerr << "Error: Destination not set. Use setDestination() or sendTo()." << std::endl;
    return -1;
  }
  return ::send(sockfd_, data.data(), data.size(), 0);
}

ssize_t UdpSocket::sendTo(const std::vector<uint8_t> & data, const std::string & host, int port)
{
  struct sockaddr_in dest_addr;
  std::memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &dest_addr.sin_addr) != 1) {
    errno = EINVAL;
    return -1;
  }

  return ::sendto(
    sockfd_, data.data(), data.size(), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

}  // namespace common
