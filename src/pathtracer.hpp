#pragma once

#include <raylib.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "rjm_raytrace_fork.h"

namespace BS {
class thread_pool;
}

class Scene;

class Pathtracer {
 private:
  RjmRayTree pathtrace_tree = {};
  std::vector<Eigen::Vector3f> pathtrace_vertices;
  std::vector<int32_t> pathtrace_indices;
  Eigen::Tensor<float, 3> pathtrace_buffer;

 private:
  [[nodiscard]]
  std::pair<Eigen::Vector3f, Eigen::Vector3f> get_ray(const RjmRay &ray) const;

  void rebuild_mesh(const Eigen::Isometry3f &transform, const Mesh &mesh);

  [[nodiscard]]
  std::vector<Eigen::Vector3f> trace_bounce(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
                                            const int32_t depth) const;

  void trace_screen(const Eigen::Vector2i &pathtrace_area, const Eigen::Matrix4f &inv_view_proj,
                    const Eigen::Vector3f &origin, const int32_t start, const int32_t end, const int32_t max_steps);

 public:
  Pathtracer() = default;
  Pathtracer(const Pathtracer &) = delete;
  Pathtracer(Pathtracer &&) = delete;
  ~Pathtracer();

  void rebuild_tree(const Scene &scene);
  void trace_image(BS::thread_pool &pool, const Camera3D &camera, const Eigen::Vector2i &pathtrace_area,
                   const int32_t steps, Image &target_image);
};