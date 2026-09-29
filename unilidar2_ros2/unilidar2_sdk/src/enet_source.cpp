#include "unilidar2_sdk/enet_source.hpp"

#include <stdexcept>

unilidar2::EnetSource::EnetSource(
  const std::string & local_ip, uint16_t local_port, const std::string & remote_ip, uint16_t remote_port)
{
  sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (sock_fd_ < 0) {
    throw std::runtime_error("socket creation failed");
  }

  memset(&local_addr_, 0, sizeof(local_addr_));
  local_addr_.sin_family = AF_INET;
  local_addr_.sin_addr.s_addr = inet_addr(local_ip.c_str());
  local_addr_.sin_port = htons(local_port);

  if (bind(sock_fd_, (const struct sockaddr *)&local_addr_, sizeof(local_addr_)) < 0) {
    // Make sure to close the socket, since the destructor won't be called
    close(sock_fd_);
    throw std::runtime_error("socket bind failed");
  }

  memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr(remote_ip.c_str());
  remote_addr_.sin_port = htons(remote_port);
}

unilidar2::EnetSource::~EnetSource() { close(sock_fd_); }

size_t unilidar2::EnetSource::get_data(uint8_t * buffer, size_t buffer_size)
{
  // Non-blocking receive to prevent the thread from being stuck when few packets are being sent.
  ssize_t n = recvfrom(sock_fd_, buffer, buffer_size, MSG_DONTWAIT, nullptr, nullptr);

  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // No data available, check again later
      return 0;
    }

    throw std::runtime_error("recvfrom failed: " + std::string(strerror(errno)));
  }

  return n;
}

void unilidar2::EnetSource::send_data(const uint8_t * data, size_t size)
{
  auto res = sendto(sock_fd_, data, size, 0, (const struct sockaddr *)&remote_addr_, sizeof(remote_addr_));

  if (res < 0) {
    throw std::runtime_error("sendto failed: " + std::string(strerror(errno)));
  }
}
