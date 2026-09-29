#include "unilidar2_sdk/lidar.hpp"

#include <zlib.h>

#include "unilidar2_sdk/messages.hpp"

namespace unilidar2
{
bool Lidar::set_work_mode(bool negative_angle)
{
  WorkModeConfigPacket packet{};
  packet.header.header[0] = FRAME_HEADER_BYTE_0;
  packet.header.header[1] = FRAME_HEADER_BYTE_1;
  packet.header.header[2] = FRAME_HEADER_BYTE_2;
  packet.header.header[3] = FRAME_HEADER_BYTE_3;
  packet.header.packet_type = WORK_MODE_CONFIG_PACKET_TYPE;
  packet.header.packet_size = sizeof(packet);

  packet.data.mode |= negative_angle ? 1 : 0;

  packet.tail.tail[0] = FRAME_TAIL_BYTE_0;
  packet.tail.tail[1] = FRAME_TAIL_BYTE_1;
  packet.tail.crc32 = crc32(0L, Z_NULL, 0);
  packet.tail.crc32 = crc32(packet.tail.crc32, reinterpret_cast<const uint8_t *>(&packet.data), sizeof(packet.data));

  return source_->send_packet(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet), true);
}

bool Lidar::sync_time(uint32_t sec, uint32_t nsec, bool block)
{
  TimeStampPacket packet{};
  packet.data.sec = sec;
  packet.data.nsec = nsec;

  packet.header.header[0] = FRAME_HEADER_BYTE_0;
  packet.header.header[1] = FRAME_HEADER_BYTE_1;
  packet.header.header[2] = FRAME_HEADER_BYTE_2;
  packet.header.header[3] = FRAME_HEADER_BYTE_3;
  packet.header.packet_type = TIME_STAMP_PACKET_TYPE;
  packet.header.packet_size = sizeof(packet);

  packet.tail.tail[0] = FRAME_TAIL_BYTE_0;
  packet.tail.tail[1] = FRAME_TAIL_BYTE_1;
  packet.tail.crc32 = crc32(0L, Z_NULL, 0);
  packet.tail.crc32 = crc32(packet.tail.crc32, reinterpret_cast<const uint8_t *>(&packet.data), sizeof(packet.data));

  return source_->send_packet(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet), block);
}

}  // namespace unilidar2