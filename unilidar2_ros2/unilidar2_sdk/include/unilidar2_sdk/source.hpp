#pragma once

#include <cstdint>
#include <cstring>

namespace unilidar2
{
class Source
{
public:
  virtual size_t get_data(uint8_t * buffer, size_t buffer_size) = 0;

  virtual void send_data(const uint8_t * data, size_t size) = 0;
};
}  // namespace unilidar2