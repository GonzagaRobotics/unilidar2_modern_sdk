#include "source.hpp"

#include "decoding.hpp"

void unilidar2::Source::rx_worker()
{
  while (running_) {
    size_t buffer_head = 0;
    size_t n;

    try {
      n = get_data(buffer_, sizeof(buffer_));
    } catch (const std::exception & e) {
      std::cerr << "Error in get_data: " << e.what() << std::endl;

      SLEEP_2;
      continue;
    }

    if (n == 0) {
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

      // TODO: Process the packet
      buffer_head += res.size + 24;  // Move past the packet header and tail
    }

    SLEEP_2;
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
