#pragma once

#include <cstring>
#include <cstdint>
#include <string>
#include <zlib.h>

#include "messages.hpp"

namespace unilidar2
{
    std::string packet_type_to_string(uint32_t packet_type);
    std::string ack_packet_to_string(const AckData *ack);

    FrameHeader parse_frame_header(const uint8_t *buffer);
    FrameTail parse_frame_tail(const uint8_t *buffer);

    bool validate_frame_ends(FrameHeader &header, FrameTail &tail);

    bool validate_crc(const uint8_t *buffer, size_t size, uint32_t expected_crc32);
} // namespace unilidar2
