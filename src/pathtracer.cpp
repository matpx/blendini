#include "pathtracer.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <BS_thread_pool.hpp>
#include <mutex>
#include <random>

#include "eigen_helper.hpp"
#include "math_helper.hpp"
#include "raymath_helper.hpp"
#include "scene.hpp"

using namespace Eigen;

[[nodiscard]]
static float rand_thread_safe(const float min, const float max) {
  static thread_local std::mt19937 generator;
  std::uniform_real_distribution<float> distribution(min, max);
  return distribution(generator);
}

[[nodiscard]]
std::pair<Vector3f, Vector3f> Pathtracer::get_ray(const RjmRay &ray) const {
  const std::array<int32_t, 3> triangle_indices = {pathtrace_indices[ray.hit * 3], pathtrace_indices[ray.hit * 3 + 1],
                                                   pathtrace_indices[ray.hit * 3 + 2]};

  const std::array<Vector3f, 3> triangle_vertices = {pathtrace_vertices[triangle_indices[0]],
                                                     pathtrace_vertices[triangle_indices[1]],
                                                     pathtrace_vertices[triangle_indices[2]]};

  const Vector3f normal =
      (triangle_vertices[1] - triangle_vertices[0]).cross(triangle_vertices[2] - triangle_vertices[0]).normalized();

  const Vector3f origin =
      (1.0f - ray.u - ray.v) * triangle_vertices[0] + ray.u * triangle_vertices[1] + ray.v * triangle_vertices[2];

  const Vector3f offset_origin = origin + normal * 0.00001f;

  return {offset_origin, normal};
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

std::vector<Vector4f> Pathtracer::trace_bounce(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch,
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

  if (depth <= 1 || hit_count == 0) {
    return light_values;
  }

  std::vector<RjmRay> next_batch(ray_batch.size());

  for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
    RjmRay next_ray = {};

    if (ray_batch[i_ray].hit != -1) {
      const auto [ray_hitpoint, ray_normal] = get_ray(ray_batch[i_ray]);

      ray_batch[i_ray].user_normal[0] = ray_normal.x();
      ray_batch[i_ray].user_normal[1] = ray_normal.y();
      ray_batch[i_ray].user_normal[2] = ray_normal.z();

      constexpr float offset_length = 0.999f;

      Vector3f dir = ray_normal + Vector3f{rand_thread_safe(-offset_length, offset_length),
                                           rand_thread_safe(-offset_length, offset_length),
                                           rand_thread_safe(-offset_length, offset_length)};

      dir.normalize();

      next_ray = {
          .org = {ray_hitpoint.x(), ray_hitpoint.y(), ray_hitpoint.z()},
          .dir = {dir.x(), dir.y(), dir.z()},
          .t = 100,
          .hit = 0,
          .u = 0,
          .v = 0,
          .visibility = 0,
          .user_normal = {},
      };
    } else {
      next_ray.t = 0.0f;
    }

    next_batch[i_ray] = next_ray;
  }

  std::vector<Vector4f> result_light_values = trace_bounce(pathtrace_tree, next_batch, depth - 1);

  for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
    if (ray_batch[i_ray].hit != -1) {
      const Vector3f ray_normal = {ray_batch[i_ray].user_normal[0], ray_batch[i_ray].user_normal[1],
                                   ray_batch[i_ray].user_normal[2]};

      light_values[i_ray] = result_light_values[i_ray] * 0.8f;
      // ray_normal.dot(Vector3f{next_batch[i_ray].dir[0], next_batch[i_ray].dir[1], next_batch[i_ray].dir[2]});
    }
  }

  return light_values;
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
        .user_normal = {},
    };
  }

  const std::vector<Vector4f> light_values = trace_bounce(pathtrace_tree, ray_batch, 4);

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

Pathtracer::Pathtracer(std::shared_ptr<ImageSwapPair> &image_swap_pair) : image_swap_pair(image_swap_pair) {}

Pathtracer::~Pathtracer() {
  if (pathtrace_tree.nodes != nullptr) {
    rjm_freeraytree(&pathtrace_tree);
  }
}

void Pathtracer::rebuild_tree(const Scene &scene) {
  sky = scene.sky;

  if (is_ready()) {
    rjm_freeraytree(&pathtrace_tree);
  }

  pathtrace_vertices.clear();
  pathtrace_indices.clear();

  for (const auto [entity, transform, mesh] : scene.view<const Isometry3f, const Mesh>().each()) {
    rebuild_mesh(transform, mesh);
  }

  for (const auto [entity, transform, model] : scene.view<const Isometry3f, const Model>().each()) {
    for (const Mesh &mesh : std::span<Mesh>(model.meshes, model.meshCount)) {
      rebuild_mesh(transform, mesh);
    }
  }

  pathtrace_tree.vtxs = reinterpret_cast<float *>(pathtrace_vertices.data());
  pathtrace_tree.tris = pathtrace_indices.data();
  pathtrace_tree.triCount = pathtrace_indices.size() / 3;

  rjm_buildraytree(&pathtrace_tree);
}

void Pathtracer::trace_image(BS::thread_pool &thread_pool, const Camera3D &camera, const Vector2i &pathtrace_area,
                             const int32_t current_step) {
  assert(pathtrace_tree.nodes != nullptr);

  const Matrix4f viewMatrix = lookAt(tr(camera.position), tr(camera.target), tr(camera.up));
  const Matrix4f projectionMatrix =
      perspective<float>(camera.fovy, ((float)pathtrace_area.x() / (float)pathtrace_area.y()), RL_CULL_DISTANCE_NEAR,
                         RL_CULL_DISTANCE_FAR);

  const Matrix4f inv_view_proj = (projectionMatrix * viewMatrix).inverse();
  const Vector3f origin = tr(camera.position);

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

      ImageDrawPixel(&image_swap_pair->write_image, screen_coords.x(), screen_coords.y(), c);
    }
  };

  const int32_t ray_count = pathtrace_area.x() * pathtrace_area.y();

  thread_pool.detach_blocks<int32_t>(0, ray_count, trace_task, ray_count / 512);
  thread_pool.wait();

  {
    std::lock_guard image_swap_pair_lock(image_swap_pair->m);
    assert(IsImageReady(image_swap_pair->write_image));

    std::swap(image_swap_pair->read_image, image_swap_pair->write_image);
  }
}