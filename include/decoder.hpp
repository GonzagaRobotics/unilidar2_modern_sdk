#pragma once

#include <iostream>
#include <stdexcept>
#include <cstring>

#include <zlib.h>

#include "messages.hpp"

#define SIZE_CHECK(s)                                                 \
    if ((size) < sizeof(s))                                           \
    {                                                                 \
        throw std::runtime_error("Insufficient data for given type"); \
    }

struct DecodeRes
{
    FrameHeader header;
    size_t bytes_parsed;
    // Pointer to the parsed data. Does not include the header or tail. The caller is responsible for freeing this memory.
    uint8_t *data;
};

bool is_valid_packet_type(uint32_t packet_type);

std::string packet_type_to_string(uint32_t packet_type);

DecodeRes decode_packet(const uint8_t *data, size_t size);