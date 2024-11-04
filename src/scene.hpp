#pragma once

#include <raylib.h>

#include <entt/entt.hpp>

class Scene final : public entt::registry {
 public:
  Camera3D camera = {
      .position = Vector3{6.0f, 6.0f, 6.0f},
      .target = Vector3{0.0f, 0.0f, 0.0f},
      .up = Vector3{0.0f, 1.0f, 0.0f},
      .fovy = 45.0f,
      .projection = CAMERA_PERSPECTIVE,
  };

 public:
  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;
  ~Scene();
};