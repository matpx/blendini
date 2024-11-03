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

  [[nodiscard]]
  std::pair<Eigen::Vector3f, Eigen::Vector3f> get_diffuse_ray(const RjmRay &output_ray) const;

  [[nodiscard]]
  float diffuse_trace(const Eigen::Vector3f &input_ray_origin, const Eigen::Vector3f &input_ray_normal,
                      const int32_t steps_left) const;

  void first_trace(const Eigen::Vector2i &pathtrace_area, const Eigen::Matrix4f &inv_view_proj,
                   const Eigen::Vector3f &origin, const int32_t start, const int32_t end, Image &target_image);

  void trace_image(BS::thread_pool &pool, const Camera3D &camera, Image &target_image,
                   const Eigen::Vector2i &pathtrace_area);
};