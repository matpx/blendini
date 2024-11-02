#pragma once

#include <raylib.h>
#include <rjm/rjm_raytrace.h>

#include <Eigen/Dense>
#include <entt/entt.hpp>

namespace BS {
class thread_pool;
}

struct Scene final : entt::registry {
  RjmRayTree pathtrace_tree = {};
  std::vector<Eigen::Vector3f> pathtrace_vertices;
  std::vector<int32_t> pathtrace_indices;

  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;

  ~Scene();

  void rebuild();
  void trace_image(BS::thread_pool &pool, const Camera3D &camera, Image &target_image,
                   const Eigen::Vector2i &pathtrace_image_size);
};