#pragma once

#include <libserial/SerialPort.h>

#include <cstdint>
#include <string>

#include "unilidar2_sdk/source.hpp"

namespace unilidar2
{
class UartSource : public Source
{
private:
  LibSerial::SerialPort serial_port_;

  // Bytes received from the serial port but not yet returned as a complete frame. The UART stream has no packet
  // boundaries, so frames are reassembled here before being handed to the decoder.
  static constexpr size_t HIDDEN_BUF_SIZE = 8192;
  uint8_t hidden_buf_[HIDDEN_BUF_SIZE];
  size_t hidden_len_ = 0;

  /// Discard the first n bytes of the hidden buffer.
  void consume(size_t n);

  size_t get_data(uint8_t * buffer, size_t buffer_size) override;

  void send_data(const uint8_t * data, size_t size) override;

public:
  UartSource(const std::string & port_name);
};
}  // namespace unilidar2