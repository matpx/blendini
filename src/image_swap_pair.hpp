#pragma once

#include <raylib-cpp.hpp>

#include <mutex>

struct ImageSwapPair {
  std::mutex swap_mutex;

  raylib::Image write_image;
  raylib::Image read_image;

  void swap() {
    std::lock_guard swap_mutex_lock(swap_mutex);
    std::swap(write_image, read_image);
  }
};