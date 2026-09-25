#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "decoding.hpp"
#include "messages.hpp"

namespace unilidar2
{
class Source
{
protected:
  std::unique_ptr<std::thread> rx_thread_;
  std::atomic<bool> running_;

  std::atomic<bool> ack_block_;

  uint8_t buffer_[8192];
  std::mutex buffer_mutex_;

  void rx_worker();

  virtual size_t get_data(uint8_t * buffer, size_t buffer_size) = 0;

  virtual void send_data(const uint8_t * data, size_t size) = 0;

public:
  bool send_packet(const uint8_t * data, size_t size, bool blocking);
};
}  // namespace unilidar2