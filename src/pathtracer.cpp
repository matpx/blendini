#include "pathtracer.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <BS_thread_pool.hpp>
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

void Pathtracer::trace_bounce(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &ray_batch, const int32_t depth,
                              std::vector<float> &light_values) const {
  const int32_t hit_count =
      rjm_raytrace(&pathtrace_tree, ray_batch.size(), ray_batch.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);

  if (depth <= 1 || hit_count == 0) {
    for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
      if (ray_batch[i_ray].hit != -1) {
        light_values[i_ray] = 0.0f;
      } else {
        light_values[i_ray] = 1.0f;
      }
    }

    return;
  }

  std::vector<RjmRay> next_batch(ray_batch.size());

  for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
    RjmRay next_ray = {};

    if (ray_batch[i_ray].hit != -1) {
      const auto [ray_hitpoint, ray_normal] = get_ray(ray_batch[i_ray]);

      ray_batch[i_ray].user_normal[0] = ray_normal.x();
      ray_batch[i_ray].user_normal[1] = ray_normal.y();
      ray_batch[i_ray].user_normal[2] = ray_normal.z();

      Vector3f dir =
          Vector3f{rand_thread_safe(-1.0f, 1.0f), rand_thread_safe(-1.0f, 1.0f), rand_thread_safe(-1.0f, 1.0f)}
              .normalized();

      if (dir.dot(ray_normal) < 0) {
        dir *= -1;
      }

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
    }

    next_batch[i_ray] = next_ray;
  }

  trace_bounce(pathtrace_tree, next_batch, depth - 1, light_values);

  for (size_t i_ray = 0; i_ray < ray_batch.size(); i_ray++) {
    if (ray_batch[i_ray].hit != -1) {
      const Vector3f ray_normal = {ray_batch[i_ray].user_normal[0], ray_batch[i_ray].user_normal[1],
                                   ray_batch[i_ray].user_normal[2]};

      light_values[i_ray] *=
          ray_normal.dot(Vector3f{next_batch[i_ray].dir[0], next_batch[i_ray].dir[1], next_batch[i_ray].dir[2]});
    }
  }
}

void Pathtracer::trace_screen(const Vector2i &pathtrace_area, const Matrix4f &inv_view_proj, const Vector3f &origin,
                              const int32_t start, const int32_t end, const int32_t steps) {
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

  std::vector<float> light_values(ray_batch.size());
  trace_bounce(pathtrace_tree, ray_batch, 4, light_values);

  for (int i_ray = start; i_ray < end; i_ray++) {
    const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};

    const float v = std::clamp(light_values[i_ray - start], 0.0f, 1.0f);

    const Vector4f new_color{v, v, v, 255};

    const float interpolation_factor = 1.0f / steps;

    pathtrace_buffer[i_ray] = new_color * interpolation_factor + pathtrace_buffer[i_ray] * (1 - interpolation_factor);
  }
}

Pathtracer::~Pathtracer() {
  if (pathtrace_tree.nodes != nullptr) {
    rjm_freeraytree(&pathtrace_tree);
  }
}

void Pathtracer::rebuild_tree(const Scene &scene) {
  if (pathtrace_tree.nodes != nullptr) {
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
                             const int32_t steps, Image &target_image) {
  assert(IsImageReady(target_image));

  const Matrix4f viewMatrix = lookAt(tr(camera.position), tr(camera.target), tr(camera.up));
  const Matrix4f projectionMatrix =
      perspective<float>(camera.fovy, ((float)pathtrace_area.x() / (float)pathtrace_area.y()), RL_CULL_DISTANCE_NEAR,
                         RL_CULL_DISTANCE_FAR);

  const Matrix4f inv_view_proj = (projectionMatrix * viewMatrix).inverse();
  const Vector3f origin = tr(camera.position);

  pathtrace_buffer.resize(pathtrace_area.x() * pathtrace_area.y());

  const auto trace_task = [&](const int32_t start, const int32_t end) {
    trace_screen(pathtrace_area, inv_view_proj, origin, start, end, steps);

    for (int i_ray = start; i_ray < end; i_ray++) {
      const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};

      const Color c{
          .r = (uint8_t)(pathtrace_buffer[i_ray].x() * 255.0f),
          .g = (uint8_t)(pathtrace_buffer[i_ray].y() * 255.0f),
          .b = (uint8_t)(pathtrace_buffer[i_ray].z() * 255.0f),
          .a = 255,
      };

      ImageDrawPixel(&target_image, screen_coords.x(), screen_coords.y(), c);
    }
  };

  const int32_t ray_count = pathtrace_area.x() * pathtrace_area.y();

  thread_pool.detach_blocks<int32_t>(0, ray_count, trace_task, ray_count / 512);
  thread_pool.wait();
}