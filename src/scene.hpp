#pragma once

#include <raylib.h>

#include <entt/entt.hpp>

#include "world.hpp"

class Scene final : public entt::registry {
 public:
  Camera3D camera = {
      .position = Vector3{8.0f, 4.0f, 8.0f},
      .target = Vector3{0.0f, 0.0f, 0.0f},
      .up = Vector3{0.0f, 1.0f, 0.0f},
      .fovy = 45.0f,
      .projection = CAMERA_PERSPECTIVE,
  };

  World world;

 public:
  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;
  ~Scene();
};