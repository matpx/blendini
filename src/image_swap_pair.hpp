#pragma once

#include <raylib.h>
#include <mutex>

struct ImageSwapPair {
  std::mutex m;

  Image write_image;
  Image read_image;
};