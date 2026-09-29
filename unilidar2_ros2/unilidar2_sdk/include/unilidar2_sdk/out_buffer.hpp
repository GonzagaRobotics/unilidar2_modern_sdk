#pragma once

#include <memory>
#include <mutex>
#include <queue>

namespace unilidar2
{
template <typename T>
class OutBuffer
{
private:
  size_t capacity_;

  std::queue<std::unique_ptr<T>> buffer_;
  std::mutex mutex_;

public:
  OutBuffer() : capacity_(10) {}
  OutBuffer(size_t capacity) : capacity_(capacity) {}

  OutBuffer(const OutBuffer &) = delete;
  OutBuffer & operator=(const OutBuffer &) = delete;

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    while (!buffer_.empty()) {
      buffer_.pop();
    }
  }

  void push(std::unique_ptr<T> data)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.size() >= capacity_) {
      buffer_.pop();
    }

    buffer_.push(std::move(data));
  }

  std::unique_ptr<T> pop()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.empty()) {
      return nullptr;
    }

    auto data = std::move(buffer_.front());
    buffer_.pop();
    return data;
  }
};
}  // namespace unilidar2