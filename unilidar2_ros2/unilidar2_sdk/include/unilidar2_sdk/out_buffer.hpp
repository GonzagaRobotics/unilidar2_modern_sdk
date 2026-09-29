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
  int capacity_;

  std::queue<std::unique_ptr<T>> buffer_;
  std::mutex mutex_;

public:
  OutBuffer() : capacity_(10) {}
  OutBuffer(int capacity) : capacity_(capacity) {}

  OutBuffer(const OutBuffer &) = delete;
  OutBuffer & operator=(const OutBuffer &) = delete;

  void clear();
  void push(std::unique_ptr<T> data);
  std::unique_ptr<T> pop();
};
}  // namespace unilidar2