#pragma once

#include <raylib.h>

#include <mutex>

struct ImageSwapPair {
  std::mutex swap_mutex;

  Image write_image;
  Image read_image;

  void swap() {
    std::lock_guard swap_mutex_lock(swap_mutex);
    std::swap(write_image, read_image);
  }
};