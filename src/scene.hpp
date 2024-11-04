#pragma once

#include <raylib.h>
#include <rjm/rjm_raytrace.h>

#include <Eigen/Dense>
#include <entt/entt.hpp>

namespace BS {
class thread_pool;
}

class Scene final : public entt::registry {
 private:
  RjmRayTree pathtrace_tree = {};
  std::vector<Eigen::Vector3f> pathtrace_vertices;
  std::vector<int32_t> pathtrace_indices;

 private:
  [[nodiscard]]
  std::pair<Eigen::Vector3f, Eigen::Vector3f> get_ray(const RjmRay &ray) const;

  void rebuild_mesh(const Eigen::Isometry3f &transform, const Mesh &mesh);

  [[nodiscard]]
  std::vector<float> trace_bounce(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
                                  const int32_t depth) const;

  void trace_screen(const Eigen::Vector2i &pathtrace_area, const Eigen::Matrix4f &inv_view_proj,
                    const Eigen::Vector3f &origin, const int32_t start, const int32_t end, Image &target_image,
                    const int32_t steps) const;

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

  void rebuild_tree();
  void trace_image(BS::thread_pool &pool, const Camera3D &camera, Image &target_image,
                   const Eigen::Vector2i &pathtrace_area, const int32_t steps) const;
};