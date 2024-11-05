#pragma once

#include <raylib.h>
#include <rlImGui/rlImGui.h>

#include <Eigen/Dense>
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

  Eigen::Vector2i pathtrace_area = {};

  std::shared_ptr<ImageSwapPair> image_swap_pair = std::make_shared<ImageSwapPair>();

  Texture2D pathtrace_texture = {};

  Shader default_shader;
  Material default_material;

  int32_t pathtrace_steps = 0;

  GfxContext();
  ~GfxContext();
};