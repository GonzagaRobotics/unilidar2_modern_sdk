#include "unilidar2_sdk/lidar.hpp"

#include <zlib.h>

#include <iostream>

#include "unilidar2_sdk/decoding.hpp"
#include "unilidar2_sdk/messages.hpp"

namespace unilidar2
{
void Lidar::rx_worker()
{
  while (running_) {
    std::unique_lock<std::mutex> lock(mutex_);

    size_t buffer_head = 0;
    size_t n;

    try {
      n = source_->get_data(buffer_, sizeof(buffer_));
    } catch (const std::exception & e) {
      std::cerr << "Error in get_data: " << e.what() << std::endl;

      lock.unlock();
      SLEEP_2;
      continue;
    }

    if (n == 0) {
      lock.unlock();
      SLEEP_2;
      continue;
    }

    while (n - buffer_head > 0) {
      DecodeRes res;
      try {
        res = decode_packet(buffer_ + buffer_head, n - buffer_head);
      } catch (const DecodeException & e) {
        std::cerr << "Error decoding packet: " << e.what() << std::endl;
        break;
      }

      buffer_head += res.size + 24;  // Move past the packet header and tail

      if (res.packet_type == ACK_DATA_PACKET_TYPE) {
        AckData * ack = reinterpret_cast<AckData *>(res.data.get());
        std::cout << "Received ACK: " << ack_packet_to_string(ack) << std::endl;

        ack_block_ = false;  // Clear the ack_block_ flag to indicate that the ACK has been received
      } else if (ack_block_) {
        continue;  // If we're waiting for an ACK, ignore other packet types
      }
    }

    lock.unlock();
    SLEEP_2;
  }
}

bool Lidar::send_packet(const void * data, size_t size, bool blocking)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (blocking) {
    ack_block_ = true;
  }

  try {
    source_->send_data(static_cast<const uint8_t *>(data), size);
  } catch (const std::exception & e) {
    std::cerr << "Error sending packet: " << e.what() << std::endl;

    ack_block_ = false;
    return false;
  }

  // Unlock the mutex before waiting for the ACK to avoid deadlocks
  lock.unlock();

  if (!blocking) {
    return true;
  }

  auto start_t = std::chrono::steady_clock::now();

  // Wait for ack_block_ to be cleared by the rx_worker thread
  while (ack_block_) {
    auto elapsed = std::chrono::steady_clock::now() - start_t;
    if (elapsed > std::chrono::seconds(1)) {
      ack_block_ = false;
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return true;
}

Lidar::Lidar(std::unique_ptr<Source> source) : source_(std::move(source))
{
  running_ = true;
  rx_thread_ = std::make_unique<std::thread>(&Lidar::rx_worker, this);
}

Lidar::~Lidar()
{
  running_ = false;
  rx_thread_->join();
}

bool Lidar::set_work_mode(bool negative_angle)
{
  WorkModeConfigPacket packet{};
  packet.header.header[0] = FRAME_HEADER_BYTE_0;
  packet.header.header[1] = FRAME_HEADER_BYTE_1;
  packet.header.header[2] = FRAME_HEADER_BYTE_2;
  packet.header.header[3] = FRAME_HEADER_BYTE_3;
  packet.header.packet_type = WORK_MODE_CONFIG_PACKET_TYPE;
  packet.header.packet_size = sizeof(packet);

  packet.data.mode = negative_angle ? 1 : 0;

  packet.tail.tail[0] = FRAME_TAIL_BYTE_0;
  packet.tail.tail[1] = FRAME_TAIL_BYTE_1;
  uint32_t crc = crc32(0L, Z_NULL, 0);
  packet.tail.crc32 = crc32(crc, reinterpret_cast<const uint8_t *>(&packet.data), sizeof(packet.data));

  return send_packet(&packet, sizeof(packet), true);
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
  auto crc = crc32(0L, Z_NULL, 0);
  packet.tail.crc32 = crc32(crc, reinterpret_cast<const uint8_t *>(&packet.data), sizeof(packet.data));

  return send_packet(&packet, sizeof(packet), block);
}

}  // namespace unilidar2