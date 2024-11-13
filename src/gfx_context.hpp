#pragma once

#include <memory>

#include "image_swap_pair.hpp"

#if defined(PLATFORM_DESKTOP)
constexpr int32_t GLSL_VERSION = 330;
#else  // PLATFORM_ANDROID, PLATFORM_WEB
constexpr int32_t GLSL_VERSION = 100;
#endif

class GfxContext {
 public:
  GfxContext(const GfxContext &) = delete;
  GfxContext(GfxContext &&) = delete;

  raylib::Window window;

  std::shared_ptr<ImageSwapPair> image_swap_pair;

  raylib::Material default_material;

  int32_t pathtrace_steps = 0;

  GfxContext();
  ~GfxContext();
};