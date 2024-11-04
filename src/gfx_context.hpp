#pragma once

#include <raylib.h>
#include <rlImGui/rlImGui.h>

#include <Eigen/Dense>

class GfxContext {
 public:
  GfxContext(const GfxContext &) = delete;
  GfxContext(GfxContext &&) = delete;

  Eigen::Vector2i pathtrace_area = {};
  Image pathtrace_image = {};
  Texture2D pathtrace_texture = {};

  int32_t pathtrace_steps = 1;

  GfxContext();
  ~GfxContext();
};