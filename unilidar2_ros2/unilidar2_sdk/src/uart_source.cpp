#include "unilidar2_sdk/uart_source.hpp"

#include <libserial/SerialPortConstants.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "unilidar2_sdk/messages.hpp"

namespace unilidar2
{

namespace
{
constexpr uint8_t FRAME_HEADER_BYTES[4] = {
  FRAME_HEADER_BYTE_0, FRAME_HEADER_BYTE_1, FRAME_HEADER_BYTE_2, FRAME_HEADER_BYTE_3};
}  // namespace

void UartSource::consume(size_t n)
{
  n = std::min(n, hidden_len_);
  std::memmove(hidden_buf_, hidden_buf_ + n, hidden_len_ - n);
  hidden_len_ -= n;
}

size_t UartSource::get_data(uint8_t * buffer, size_t buffer_size)
{
  // Pull whatever is pending into the hidden buffer. Only the available byte count is read so this never blocks.
  size_t space = HIDDEN_BUF_SIZE - hidden_len_;
  size_t available = std::max(0, serial_port_.GetNumberOfBytesAvailable());
  size_t to_read = std::min(space, available);

  if (to_read > 0) {
    ssize_t n = read(serial_port_.GetFileDescriptor(), hidden_buf_ + hidden_len_, to_read);

    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        throw std::runtime_error("serial read failed: " + std::string(strerror(errno)));
      }
    } else {
      hidden_len_ += n;
    }
  }

  while (true) {
    // Skip ahead to the next header. A partial header match at the end of the buffer is kept so it can complete on a
    // later read.
    auto it =
      std::search(hidden_buf_, hidden_buf_ + hidden_len_, std::begin(FRAME_HEADER_BYTES), std::end(FRAME_HEADER_BYTES));
    size_t head = it - hidden_buf_;

    if (it == hidden_buf_ + hidden_len_) {
      head = hidden_len_ >= sizeof(FRAME_HEADER_BYTES) ? hidden_len_ - (sizeof(FRAME_HEADER_BYTES) - 1) : 0;
      while (head < hidden_len_ && std::memcmp(hidden_buf_ + head, FRAME_HEADER_BYTES, hidden_len_ - head) != 0) {
        ++head;
      }
    }

    consume(head);

    if (hidden_len_ < sizeof(FrameHeader)) {
      return 0;  // Need more data to read the packet size
    }

    FrameHeader header;
    std::memcpy(&header, hidden_buf_, sizeof(header));

    // A size that can't hold header + tail or doesn't fit the buffer means this was a false header, so resync past it
    if (header.packet_size < sizeof(FrameHeader) + sizeof(FrameTail) || header.packet_size > HIDDEN_BUF_SIZE) {
      consume(1);
      continue;
    }

    if (hidden_len_ < header.packet_size) {
      return 0;  // Frame not complete yet
    }

    const uint8_t * tail = hidden_buf_ + header.packet_size - 2;
    if (tail[0] != FRAME_TAIL_BYTE_0 || tail[1] != FRAME_TAIL_BYTE_1) {
      consume(1);
      continue;
    }

    size_t frame_size = header.packet_size;
    if (frame_size > buffer_size) {
      consume(frame_size);
      throw std::runtime_error("output buffer too small for frame");
    }

    std::memcpy(buffer, hidden_buf_, frame_size);
    consume(frame_size);

    return frame_size;
  }
}

void UartSource::send_data(const uint8_t * data, size_t size) { write(serial_port_.GetFileDescriptor(), data, size); }

UartSource::UartSource(const std::string & port_name)
{
  serial_port_.Open(port_name);
  serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_4000000);
  serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
}

}  // namespace unilidar2
