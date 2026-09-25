#pragma once

#include "source.hpp"
#include "messages.hpp"

namespace unilidar2
{
    class Lidar
    {
    private:
        std::unique_ptr<Source> source_;

    public:
        Lidar(std::unique_ptr<Source> source) : source_(std::move(source)) {}

        ~Lidar();
    };
}