#include "unilidar2_sdk/decoding.hpp"

namespace unilidar2
{
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

    std::string ack_packet_to_string(const AckData *ack)
    {
        std::string status;
        switch (ack->status)
        {
        case ACK_SUCCESS:
            status = "ACK_SUCCESS";
            break;
        case ACK_CRC_ERROR:
            status = "ACK_CRC_ERROR";
            break;
        case ACK_HEADER_ERROR:
            status = "ACK_HEADER_ERROR";
            break;
        case ACK_BLOCK_ERROR:
            status = "ACK_BLOCK_ERROR";
            break;
        case ACK_WAIT_ERROR:
            status = "ACK_WAIT_ERROR";
            break;
        default:
            status = "Unknown (" + std::to_string(ack->status) + ")";
            break;
        }

        return "ACK packet_type: " + std::to_string(ack->packet_type) +
               ", cmd_type: " + std::to_string(ack->cmd_type) +
               ", cmd_value: " + std::to_string(ack->cmd_value) +
               ", status: " + status;
    }

    FrameHeader parse_frame_header(const uint8_t *buffer)
    {
        FrameHeader header;
        std::memcpy(&header, buffer, 12);

        return header;
    }

    FrameTail parse_frame_tail(const uint8_t *buffer)
    {
        FrameTail tail;
        std::memcpy(&tail, buffer, 12);

        return tail;
    }

    bool validate_frame_ends(FrameHeader &header, FrameTail &tail)
    {
        if (header.header[0] != FRAME_HEADER_BYTE_0 ||
            header.header[1] != FRAME_HEADER_BYTE_1 ||
            header.header[2] != FRAME_HEADER_BYTE_2 ||
            header.header[3] != FRAME_HEADER_BYTE_3)
        {
            return false;
        }

        // Packet header + tail is 24 bytes, so the packet size must be at least that large
        if (header.packet_size < 24)
        {
            return false;
        }

        // TODO: Expand this list over time
        if (header.packet_type != POINT_DATA_PACKET_TYPE &&
            header.packet_type != IMU_DATA_PACKET_TYPE &&
            header.packet_type != ACK_DATA_PACKET_TYPE)
        {
            return false;
        }

        if (tail.tail[0] != FRAME_TAIL_BYTE_0 ||
            tail.tail[1] != FRAME_TAIL_BYTE_1)
        {
            return false;
        }

        return true;
    }

    bool validate_crc(const uint8_t *buffer, size_t size, uint32_t expected_crc32)
    {
        uint32_t computed_crc = crc32(0L, Z_NULL, 0);
        computed_crc = crc32(computed_crc, buffer, size);

        return computed_crc == expected_crc32;
    }
} // namespace unilidar2
