#pragma once

#include <raylib.h>
#include <rjm/rjm_raytrace.h>

#include <Eigen/Dense>
#include <entt/entt.hpp>
#include <vector>

struct Scene final : entt::registry {
  RjmRayTree tree = {};

  std::vector<Eigen::Vector3f> vertices;
  std::vector<int32_t> indices;

  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;

  ~Scene();

  void rebuild();
  void trace_image(const Camera3D &camera, Image &target_image);
};