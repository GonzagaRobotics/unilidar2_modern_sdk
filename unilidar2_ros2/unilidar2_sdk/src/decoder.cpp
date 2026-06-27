#include "decoder.hpp"

std::string packet_type_to_string(uint32_t packet_type)
{
    switch (packet_type)
    {
    case POINT_DATA_PACKET_TYPE:
        return "PointDataPacket";
    case IMU_DATA_PACKET_TYPE:
        return "ImuDataPacket";
    case ACK_DATA_PACKET_TYPE:
        return "AckDataPacket";
    default:
        return "Invalid (" + std::to_string(packet_type) + ")";
    }
}

bool is_valid_packet_type(uint32_t packet_type)
{
    // TODO: Add more packet types as they are implemented
    if (packet_type == POINT_DATA_PACKET_TYPE ||
        packet_type == IMU_DATA_PACKET_TYPE ||
        packet_type == ACK_DATA_PACKET_TYPE)
    {
        return true;
    }

    return false;
}

DecodeRes decode_packet(const uint8_t *data, size_t size)
{
    if ((size) < sizeof(FrameHeader))
    {
        throw std::runtime_error("Insufficient data for FrameHeader");
    }

    FrameHeader header;
    std::memcpy(&header, data, sizeof(FrameHeader));

    // Sanity check the header
    if (header.header[0] != FRAME_HEADER_BYTE_0 || header.header[1] != FRAME_HEADER_BYTE_1 ||
        header.header[2] != FRAME_HEADER_BYTE_2 || header.header[3] != FRAME_HEADER_BYTE_3)
    {
        throw std::runtime_error("Packet header has corrupted start bytes");
    }
    if (header.packet_size > size)
    {
        // Sometimes, the packet is split between two UDP packets, so we may
        // not have the full packet yet. So we return a special result indicating that we need more data.
        return DecodeRes{header, 0, nullptr};
    }
    if (!is_valid_packet_type(header.packet_type))
    {
        throw std::runtime_error("Invalid packet type: " + std::to_string(header.packet_type));
    }

    FrameTail tail;
    std::memcpy(&tail, data + header.packet_size - sizeof(FrameTail), sizeof(FrameTail));

    // Sanity check the tail
    if (tail.tail[0] != FRAME_TAIL_BYTE_0 || tail.tail[1] != FRAME_TAIL_BYTE_1)
    {
        throw std::runtime_error("Packet tail has corrupted end bytes " + std::to_string(tail.tail[0]) + " " + std::to_string(tail.tail[1]));
    }

    size_t data_size = header.packet_size - sizeof(FrameHeader) - sizeof(FrameTail);
    auto data_start = data + sizeof(FrameHeader);

    // Verify the CRC32 checksum
    uint32_t computed_crc = crc32(0L, Z_NULL, 0);
    computed_crc = crc32(computed_crc, data_start, data_size);

    if (computed_crc != tail.crc32)
    {
        throw std::runtime_error("Packet CRC32 checksum mismatch");
    }

    DecodeRes res;
    res.header = header;
    res.bytes_parsed = header.packet_size;
    res.data = std::make_unique<uint8_t[]>(data_size);
    std::memcpy(res.data.get(), data_start, data_size);

    return res;
}