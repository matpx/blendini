#pragma once

#include <raylib.h>
#include <rlImGui/rlImGui.h>

#include <Eigen/Dense>

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
  Image pathtrace_image = {};
  Texture2D pathtrace_texture = {};

  Shader default_shader;
  Material default_material;

  int32_t pathtrace_steps = 1;

  GfxContext();
  ~GfxContext();
};