#pragma once

#include <Eigen/Dense>
#include <future>
#include <unsupported/Eigen/CXX11/Tensor>

#include "image_swap_pair.hpp"
#include "rjm_raytrace.hpp"
#include "sky.hpp"

namespace BS {
class thread_pool;
}

class Scene;

class Pathtracer {
 public:
  enum class Status {
    STOPPED,
    RUNNING,
    FINISHED,
    ABORT,
  };

 private:
  std::shared_ptr<BS::thread_pool> pool;
  std::shared_ptr<ImageSwapPair> image_swap_pair;

  RjmRayTree pathtrace_tree = {};
  std::vector<Eigen::Vector3f> pathtrace_vertices;
  std::vector<int32_t> pathtrace_indices;
  Eigen::Tensor<float, 3> pathtrace_buffer;
  Sky sky = {};

  std::future<void> render_future;

 private:
  void rebuild_mesh(const Eigen::Isometry3f &transform, const Mesh &mesh);

  [[nodiscard]]
  std::vector<Eigen::Vector4f> trace_rays(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
                                          const int32_t depth) const;

  [[nodiscard]]
  std::vector<Eigen::Vector4f> trace_bounce(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
                                            const int32_t depth) const;

  void trace_screen(const Eigen::Vector2i &pathtrace_area, const Eigen::Matrix4f &inv_view_proj,
                    const Eigen::Vector3f &origin, const int32_t start, const int32_t end, const int32_t current_step);

  void trace_image(const Eigen::Matrix4f &inv_view_proj, const Eigen::Vector3f &origin,
                   const Eigen::Vector2i &pathtrace_area, const int32_t current_step);

 public:
  Pathtracer(std::shared_ptr<BS::thread_pool> &pool, std::shared_ptr<ImageSwapPair> &image_swap_pair);
  Pathtracer(const Pathtracer &) = delete;
  Pathtracer(Pathtracer &&) = delete;
  ~Pathtracer();

  Status status = Status::STOPPED;

  void rebuild_tree(const Scene &scene);

  void stop();
  void start(const raylib::Camera3D &camera, const Eigen::Vector2i &pathtrace_area, const int32_t steps);
};