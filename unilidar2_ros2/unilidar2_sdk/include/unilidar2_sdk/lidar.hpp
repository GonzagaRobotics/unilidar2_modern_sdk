#pragma once

#include "source.hpp"

namespace unilidar2
{
class Lidar
{
private:
  std::unique_ptr<Source> source_;

public:
  Lidar(std::unique_ptr<Source> source) : source_(std::move(source)) {}
  Lidar(const Lidar &) = delete;
  Lidar & operator=(const Lidar &) = delete;

  ~Lidar() {};

  bool set_work_mode(bool negative_angle);
  bool sync_time(uint32_t sec, uint32_t nsec, bool block = false);
};
}  // namespace unilidar2