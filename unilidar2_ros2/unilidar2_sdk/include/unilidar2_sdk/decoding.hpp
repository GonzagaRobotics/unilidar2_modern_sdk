#pragma once

#include <zlib.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "messages.hpp"

namespace unilidar2
{
class DecodeException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

struct DecodeRes
{
  uint32_t packet_type;
  size_t size;
  std::unique_ptr<uint8_t[]> data;
};

DecodeRes decode_packet(const uint8_t * buffer, size_t size);

std::string packet_type_to_string(uint32_t packet_type);
std::string ack_packet_to_string(const AckData * ack);

FrameHeader parse_frame_header(const uint8_t * buffer);
FrameTail parse_frame_tail(const uint8_t * buffer);

bool validate_frame_ends(FrameHeader & header, FrameTail & tail);

bool validate_crc(const uint8_t * buffer, size_t size, uint32_t expected_crc32);
}  // namespace unilidar2
