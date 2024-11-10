#pragma once

#include <entt/entt.hpp>
#include <raylib-cpp.hpp>

#include "sky.hpp"

class Scene final : public entt::registry {
 public:
  raylib::Camera3D camera = {
      Vector3{8.0f, 4.0f, 8.0f}, Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 1.0f, 0.0f}, 45.0f, CAMERA_PERSPECTIVE,
  };

  Sky sky = {};

 public:
  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;
};