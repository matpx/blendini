#include "pathtracer.hpp"

#include <rlgl.h>

#include <BS_thread_pool.hpp>

#include "eigen_helper.hpp"
#include "raymath_helper.hpp"
#include "scene.hpp"

using namespace Eigen;

constexpr float SMALL_NUMBER = std::numeric_limits<float>::epsilon() * 100.0f;

static_assert(std::atomic<Pathtracer::Status>::is_always_lock_free,
              "!std::atomic<Pathtracer::Status>::is_always_lock_free");
static_assert(std::atomic<int32_t>::is_always_lock_free, "!std::atomic<int32_t>::is_always_lock_free");

[[nodiscard]]
static float rand_thread_safe(const float min, const float max) {
  static thread_local std::mt19937 generator;
  std::uniform_real_distribution<float> distribution(min, max);
  return distribution(generator);
}

[[nodiscard]]
inline static std::pair<Vector3f, Vector3f> new_origin_and_dir(const RjmRay &ray) {
  constexpr float offset_length = 1.0f - SMALL_NUMBER;

  const Vector3f origin = Vector3f{ray.org[0], ray.org[1], ray.org[2]};
  const Vector3f dir = Vector3f{ray.dir[0], ray.dir[1], ray.dir[2]};
  const Vector3f normal = Vector3f{ray.normal[0], ray.normal[1], ray.normal[2]}.normalized();
  const Vector3f sphere_offset =
      Vector3f{rand_thread_safe(-offset_length, offset_length), rand_thread_safe(-offset_length, offset_length),
               rand_thread_safe(-offset_length, offset_length)};

  const Vector3f new_origin = (origin + dir * ray.t) + normal * SMALL_NUMBER;
  const Vector3f new_dir = (normal + sphere_offset).normalized();

  return {new_origin, new_dir};
}

void Pathtracer::rebuild_mesh(const Isometry3f &transform, const Mesh &mesh) {
  const uint16_t last_vertex = pathtrace_vertices.size();

  static_assert(sizeof(Vector3f) == sizeof(float) * 3);

  for (const auto &vertex : std::span<Vector3f>(reinterpret_cast<Vector3f *>(mesh.vertices), mesh.vertexCount)) {
    pathtrace_vertices.push_back(transform * vertex);
  }

  if (mesh.indices != nullptr) {
    for (const auto &index : std::span<uint16_t>(mesh.indices, mesh.triangleCount * 3)) {
      pathtrace_indices.push_back(last_vertex + index);
    }
  } else {
    for (int32_t i_index = 0; i_index < mesh.triangleCount * 3; i_index++) {
      pathtrace_indices.push_back(last_vertex + i_index);
    }
  }
}

std::vector<Vector4f> Pathtracer::trace_rays(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
                                             const int32_t depth) const {
  const int32_t hit_count =
      rjm_raytrace(&pathtrace_tree, ray_batch.size(), ray_batch.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);

  std::vector<Vector4f> light_values(ray_batch.size());

  for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
    if (ray_batch[i_ray].hit != -1) {
      light_values[i_ray] = Vector4f{0.0f, 0.0f, 0.0f, 0.0f};
    } else {
      const float sky_blend_factor = ray_batch[i_ray].dir[1] * 0.5f + 0.5f;
      light_values[i_ray] = (1.0 - sky_blend_factor) * sky.bottom_color + sky_blend_factor * sky.top_color;
    }
  }

  if (depth > 0 && hit_count > 0) {
    const std::vector<Vector4f> result_light_values = trace_bounce(pathtrace_tree, ray_batch, depth - 1);

    for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
      if (ray_batch[i_ray].hit != -1) {
        light_values[i_ray] = result_light_values[i_ray] * 0.8f;
      }
    }
  }

  return light_values;
}

std::vector<Vector4f> Pathtracer::trace_bounce(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
                                               const int32_t depth) const {
  std::vector<RjmRay> next_batch(ray_batch.size());

  for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
    const RjmRay &ray = ray_batch[i_ray];

    RjmRay next_ray = {};

    if (ray.hit != -1) {
      const auto [new_origin, new_dir] = new_origin_and_dir(ray);

      next_ray = {
          .org = {new_origin.x(), new_origin.y(), new_origin.z()},
          .dir = {new_dir.x(), new_dir.y(), new_dir.z()},
          .t = 100,
          .hit = 0,
          .u = 0,
          .v = 0,
          .visibility = 0,
          .normal = {},
      };
    } else {
      next_ray.t = 0.0f;
    }

    next_batch[i_ray] = next_ray;
  }

  return trace_rays(pathtrace_tree, next_batch, depth);
}

void Pathtracer::trace_screen(const Vector2i &pathtrace_area, const Matrix4f &inv_view_proj, const Vector3f &origin,
                              const int32_t start, const int32_t end, const int32_t current_step) {
  assert(start < end);

  std::vector<RjmRay> ray_batch(end - start);

  for (int i_ray = start; i_ray < end; i_ray++) {
    constexpr float jiggle_width = 0.5f;

    Vector2f screen_coords =
        Vector2f{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()} +
        Vector2f{rand_thread_safe(-jiggle_width, jiggle_width), rand_thread_safe(-jiggle_width, jiggle_width)};

    const Vector3f ray = screen_to_world(screen_coords, pathtrace_area, inv_view_proj);

    ray_batch[i_ray - start] = {
        .org = {origin.x(), origin.y(), origin.z()},
        .dir = {ray.x(), ray.y(), ray.z()},
        .t = 100,
        .hit = 0,
        .u = 0,
        .v = 0,
        .visibility = 0,
        .normal = {},
    };
  }

  const std::vector<Vector4f> light_values = trace_rays(pathtrace_tree, ray_batch, 4);

  const float interpolation_factor = 1.0f / (current_step + 1);

  for (int i_ray = start; i_ray < end; i_ray++) {
    const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};

    const Vector4f new_color = light_values[i_ray - start].cwiseMax(0.0f).cwiseMin(1.0f);

    for (int32_t i_channel = 0; i_channel < pathtrace_buffer.dimension(2); i_channel++) {
      pathtrace_buffer(screen_coords.x(), screen_coords.y(), i_channel) =
          new_color[i_channel] * interpolation_factor +
          pathtrace_buffer.coeff(screen_coords.x(), screen_coords.y(), i_channel) * (1 - interpolation_factor);
    }
  }
}

void Pathtracer::trace_image(const Matrix4f &inv_view_proj, const Vector3f &origin, const Vector2i &pathtrace_area,
                             const int32_t current_step) {
  pathtrace_buffer.resize(pathtrace_area.x(), pathtrace_area.y(), 3);

  const auto trace_task = [&](const int32_t start, const int32_t end) {
    trace_screen(pathtrace_area, inv_view_proj, origin, start, end, current_step);

    for (int i_ray = start; i_ray < end; i_ray++) {
      const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};

      const Color c{
          .r = (uint8_t)(pathtrace_buffer.coeff(screen_coords.x(), screen_coords.y(), 0) * 255.0f),
          .g = (uint8_t)(pathtrace_buffer.coeff(screen_coords.x(), screen_coords.y(), 1) * 255.0f),
          .b = (uint8_t)(pathtrace_buffer.coeff(screen_coords.x(), screen_coords.y(), 2) * 255.0f),
          .a = 255,
      };

      image_swap_pair->write_image.DrawPixel(screen_coords.x(), screen_coords.y(), c);
    }
  };

  const int32_t ray_count = pathtrace_area.x() * pathtrace_area.y();

  pool->detach_blocks<int32_t>(0, ray_count, trace_task, ray_count / 1024);
  pool->wait();

  image_swap_pair->swap();
}

Pathtracer::Pathtracer(std::shared_ptr<BS::thread_pool> &pool, std::shared_ptr<ImageSwapPair> &image_swap_pair)
    : pool(pool), image_swap_pair(image_swap_pair) {}

Pathtracer::~Pathtracer() {
  stop_and_join();

  if (pathtrace_tree.nodes != nullptr) {
    rjm_freeraytree(&pathtrace_tree);
  }
}

void Pathtracer::rebuild_tree(const Scene &scene) {
  stop_and_join();

  sky = scene.sky;

  if (pathtrace_tree.nodes != nullptr) {
    rjm_freeraytree(&pathtrace_tree);
  }

  pathtrace_vertices.clear();
  pathtrace_indices.clear();

  for (const auto [entity, transform, mesh] :
       scene.view<const Isometry3f, const std::shared_ptr<raylib::Mesh>>().each()) {
    rebuild_mesh(transform, *mesh);
  }

  for (const auto [entity, transform, model] :
       scene.view<const Isometry3f, const std::shared_ptr<raylib::Model>>().each()) {
    for (const Mesh &mesh : std::span<Mesh>(model->GetMeshes(), model->GetMeshCount())) {
      rebuild_mesh(transform, mesh);
    }
  }

  pathtrace_tree.vtxs = reinterpret_cast<float *>(pathtrace_vertices.data());
  pathtrace_tree.tris = pathtrace_indices.data();
  pathtrace_tree.triCount = pathtrace_indices.size() / 3;

  rjm_buildraytree(&pathtrace_tree);
}

void Pathtracer::stop_and_join() {
  status = Status::ABORT;
  if (render_future.valid()) {
    render_future.wait();
  }
  status = Status::STOPPED;
}

void Pathtracer::start(const raylib::Camera3D &camera, const Eigen::Vector2i &pathtrace_area,
                       const int32_t max_pathtrace_step) {
  stop_and_join();

  if (pathtrace_tree.nodes == nullptr || !(IsImageReady(image_swap_pair->write_image))) {
    assert(false);
    return;
  }

  status = Status::RUNNING;
  current_step = 0;

  image_swap_pair->write_image.ClearBackground(raylib::Color{0, 0, 0, 0});
  image_swap_pair->swap();
  image_swap_pair->write_image.ClearBackground(raylib::Color{0, 0, 0, 0});

  render_future = std::async(std::launch::async, [=, this]() {
    for (int32_t i_step = 0; i_step < max_pathtrace_step; i_step++) {
      if (status == Status::ABORT) {
        status = Status::STOPPED;
        return;
      }

      const Matrix4f viewMatrix = lookAt(tr(camera.position), tr(camera.target), tr(camera.up));
      const Matrix4f projectionMatrix =
          perspective<float>(camera.fovy, ((float)pathtrace_area.x() / (float)pathtrace_area.y()),
                             RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);

      const Matrix4f inv_view_proj = (projectionMatrix * viewMatrix).inverse();
      const Vector3f origin = tr(camera.position);

      trace_image(inv_view_proj, origin, pathtrace_area, i_step);

      current_step++;
    }

    status = Status::FINISHED;
  });
}
