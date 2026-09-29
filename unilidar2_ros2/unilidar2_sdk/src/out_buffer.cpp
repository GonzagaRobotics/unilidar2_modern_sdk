#include "unilidar2_sdk/out_buffer.hpp"

namespace unilidar2
{
template <typename T>
void OutBuffer<T>::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);

  while (!buffer_.empty()) {
    buffer_.pop();
  }
}

template <typename T>
void OutBuffer<T>::push(std::unique_ptr<T> data)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.size() >= capacity_) {
    buffer_.pop();
  }

  buffer_.push(std::move(data));
}

template <typename T>
std::unique_ptr<T> OutBuffer<T>::pop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty()) {
    return nullptr;
  }

  auto data = std::move(buffer_.front());
  buffer_.pop();
  return data;
}

}  // namespace unilidar2