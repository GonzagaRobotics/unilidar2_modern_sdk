#include "source.hpp"

void unilidar2::Source::rx_worker()
{
  while (running_) {
    try {
      size_t n = get_data(buffer_, sizeof(buffer_));

      if (n > 0 && n < 12) {
        std::cout << "too short: " << std::hex;
        for (size_t i = 0; i < n; i++) {
          std::cout << buffer_[i];
        }

        std::cout << std::dec << std::endl;
      } else if (n >= 12) {
        FrameHeader header = parse_frame_header(buffer_);
        std::cout << "got " << packet_type_to_string(header.packet_type) << " " << header.packet_size << " bytes"
                  << std::endl;
        if (n >= header.packet_size) {
          FrameTail tail = parse_frame_tail(buffer_ + header.packet_size - 12);
          if (!validate_frame_ends(header, tail)) {
            std::cout << "invalid frame" << std::endl;
          }

          if (!validate_crc(buffer_ + 12, header.packet_size - 24, tail.crc32)) {
            std::cout << "invalid CRC" << std::endl;
          }

          if (n - header.packet_size > 0) {
            std::cout << "extra data after frame: " << n - header.packet_size << " bytes" << std::endl;
          }
        } else {
          std::cout << "incomplete frame expected " << header.packet_size << " got " << n << std::endl;
        }
      }
    } catch (const std::exception & e) {
      std::cerr << "Error in rx_worker: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool unilidar2::Source::send_packet(const uint8_t * data, size_t size, bool blocking)
{
  if (blocking) {
    ack_block_ = true;
  }

  try {
    send_data(data, size);
  } catch (const std::exception & e) {
    std::cerr << "Error sending packet: " << e.what() << std::endl;

    ack_block_ = false;
    return false;
  }

  if (!blocking) {
    return true;
  }

  auto start_t = std::chrono::steady_clock::now();

  // Wait for ack_block_ to be cleared by the rx_worker thread
  while (ack_block_) {
    auto elapsed = std::chrono::steady_clock::now() - start_t;
    if (elapsed > std::chrono::seconds(1)) {
      std::cerr << "Timeout waiting for ack" << std::endl;
      ack_block_ = false;
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return true;
}
