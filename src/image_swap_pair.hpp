#pragma once

#include <raylib-cpp.hpp>

#include "math_helper.hpp"

struct ImageSwapPair {
  std::mutex swap_mutex;

  Eigen::Vector2i size;

  raylib::Image write_image;
  raylib::Image read_image;

  raylib::Texture2D pathtrace_texture;

  ImageSwapPair(const ImageSwapPair &) = delete;
  ImageSwapPair(ImageSwapPair &&) = delete;
  ImageSwapPair(const Eigen::Vector2i &size)
      : size(size),
        write_image(next_power_of_two(size.x()), next_power_of_two(size.y()), BLACK),
        read_image(write_image),
        pathtrace_texture(read_image) {}

  void swap() {
    std::lock_guard swap_mutex_lock(swap_mutex);
    std::swap(write_image, read_image);
  }

  void update_texture() {
    Color *pathtrace_image_colors = nullptr;
    {
      std::lock_guard lock(swap_mutex);
      pathtrace_image_colors = read_image.LoadColors();
    }

    pathtrace_texture.Update(pathtrace_image_colors);
    read_image.UnloadColors(pathtrace_image_colors);
  }
};