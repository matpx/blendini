#include "scene.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <raylib.h>
#include <rlgl.h>

#include <BS_thread_pool.hpp>

#include "raymath_eigen.hpp"

using namespace Eigen;

[[nodiscard]]
inline static Vector3f screen_to_world(const Vector2i &screen_pos, const Vector2i &screen_size,
                                       const Matrix4f &inv_view_proj) {
  const Vector2f ndc{
      (2.0f * screen_pos.x()) / screen_size.x() - 1.0f,
      1.0f - (2.0f * screen_pos.y()) / screen_size.y(),
  };

  const Vector4f direction = inv_view_proj * Vector4f(ndc.x(), ndc.y(), 1.0f, 1.0f);

  return direction.head<3>();
}

[[nodiscard]]
static std::pair<Vector3f, Vector3f> get_ray(const std::vector<Eigen::Vector3f> &pathtrace_vertices,
                                             const std::vector<int32_t> &pathtrace_indices, const RjmRay &ray) {
  const std::array<int32_t, 3> triangle_indices = {pathtrace_indices[ray.hit * 3], pathtrace_indices[ray.hit * 3 + 1],
                                                   pathtrace_indices[ray.hit * 3 + 2]};

  const std::array<Vector3f, 3> triangle_vertices = {pathtrace_vertices[triangle_indices[0]],
                                                     pathtrace_vertices[triangle_indices[1]],
                                                     pathtrace_vertices[triangle_indices[2]]};

  const Vector3f normal =
      (triangle_vertices[1] - triangle_vertices[0]).cross(triangle_vertices[2] - triangle_vertices[0]);

  const Vector3f origin =
      (1.0f - ray.u - ray.v) * triangle_vertices[0] + ray.u * triangle_vertices[1] + ray.v * triangle_vertices[2];

  const Vector3f offset_origin = origin + normal * 0.01f;

  return {offset_origin, normal};
}

Scene::~Scene() {
  if (pathtrace_tree.nodes != nullptr) {
    rjm_freeraytree(&pathtrace_tree);
  }

  for (const auto [entity, mesh] : view<const Mesh>().each()) {
    if (mesh.vaoId > 0) {
      UnloadMesh(mesh);
    }
  }
}

void Scene::rebuild() {
  if (pathtrace_tree.nodes != nullptr) {
    rjm_freeraytree(&pathtrace_tree);
  }

  pathtrace_vertices.clear();
  pathtrace_indices.clear();

  const auto rebuild_mesh = [&](const Isometry3f &transform, const Mesh &mesh) {
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
  };

  for (const auto [entity, transform, mesh] : view<const Isometry3f, const Mesh>().each()) {
    rebuild_mesh(transform, mesh);
  }

  for (const auto [entity, transform, model] : view<const Isometry3f, const Model>().each()) {
    for (const Mesh &mesh : std::span<Mesh>(model.meshes, model.meshCount)) {
      rebuild_mesh(transform, mesh);
    }
  }

  pathtrace_tree.vtxs = reinterpret_cast<float *>(pathtrace_vertices.data());
  pathtrace_tree.tris = pathtrace_indices.data();
  pathtrace_tree.triCount = pathtrace_indices.size() / 3;

  rjm_buildraytree(&pathtrace_tree);
}

// float Scene::diffuse_trace(const Vector3f &input_ray_origin, const Vector3f &input_ray_normal,
//                            const int32_t steps_left) const {
//   if (steps_left <= 0) {
//     return 0.0f;
//   }

//   Vector3f diffuse_dir = Vector3f::Random();

//   if (diffuse_dir.dot(input_ray_normal) < 0) {
//     diffuse_dir *= -1;
//   }

//   RjmRay ray = {
//       .org = {input_ray_origin.x(), input_ray_origin.y(), input_ray_origin.z()},
//       .dir = {diffuse_dir.x(), diffuse_dir.y(), diffuse_dir.z()},
//       .t = 100,
//       .hit = 0,
//       .u = 0,
//       .v = 0,
//       .visibility = 0,
//   };

//   rjm_raytrace(&pathtrace_tree, 1, &ray, RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);

//   const auto cos_factor = [](const Vector3f &a, const Vector3f &b) -> float {
//     float dotProduct = a.dot(a);

//     float magnitudeA = a.norm();
//     float magnitudeB = b.norm();

//     return dotProduct / (magnitudeA * magnitudeB);
//   };

//   float lux = 0.8f;

//   if (ray.hit != -1) {
//     const auto [diffuse_ray_origin, diffuse_ray_normal] = get_ray(ray);
//     lux = diffuse_trace(diffuse_ray_origin, diffuse_ray_normal, steps_left - 1);
//     lux *= std::abs(cos_factor(diffuse_ray_normal, Vector3f{ray.dir[0], ray.dir[1], ray.dir[2]}));
//   }

//   return lux;
// };

std::vector<float> Scene::trace_batch(const RjmRayTree &pathtrace_tree, std::vector<RjmRay> &rays,
                                      const int32_t depth) {
  rjm_raytrace(&pathtrace_tree, rays.size(), rays.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);

  std::vector<float> lux_values(rays.size());
  std::vector<RjmRay> next_batch(rays.size());

  bool atleast_one_hit = false;

  for (size_t i_ray = 0; i_ray < rays.size(); i_ray++) {
    RjmRay next_ray = {};

    if (rays[i_ray].hit != -1) {
      const auto [ray_origin, ray_normal] = get_ray(pathtrace_vertices, pathtrace_indices, rays[i_ray]);

      Vector3f dir = Vector3f::Random();

      if (dir.dot(ray_normal) < 0) {
        dir *= -1;
      }

      next_ray = {
          .org = {ray_origin.x(), ray_origin.y(), ray_origin.z()},
          .dir = {dir.x(), dir.y(), dir.z()},
          .t = 100,
          .hit = 0,
          .u = 0,
          .v = 0,
          .visibility = 0,
      };

      lux_values[i_ray] = 0.0f;

      atleast_one_hit = true;
    } else {
      lux_values[i_ray] = 0.4f;
    }

    next_batch[i_ray] = next_ray;
  }

  if (depth <= 0 || !atleast_one_hit) {
    return lux_values;
  }

  std::vector<float> result_lux_values = trace_batch(pathtrace_tree, next_batch, depth - 1);

  const auto cos_factor = [](const Vector3f &a, const Vector3f &b) -> float {
    float dotProduct = a.dot(a);

    float magnitudeA = a.norm();
    float magnitudeB = b.norm();

    return dotProduct / (magnitudeA * magnitudeB);
  };

  for (size_t i_ray = 0; i_ray < rays.size(); i_ray++) {
    if (rays[i_ray].hit != -1) {
      const auto [ray_origin, ray_normal] = get_ray(pathtrace_vertices, pathtrace_indices, rays[i_ray]);  // TODO

      lux_values[i_ray] = result_lux_values[i_ray] *
                          cos_factor(ray_normal, Vector3f{rays[i_ray].dir[0], rays[i_ray].dir[1], rays[i_ray].dir[2]});
    }
  }

  return lux_values;
}

void Scene::first_trace(const Vector2i &pathtrace_area, const Matrix4f &inv_view_proj, const Vector3f &origin,
                        const int32_t start, const int32_t end, Image &target_image, const bool reset) {
  assert(start < end);

  std::vector<RjmRay> rays(end - start);

  for (int i_ray = start; i_ray < end; i_ray++) {
    const Vector2i screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};
    const Vector3f ray = screen_to_world(screen_coords, pathtrace_area, inv_view_proj);

    rays[i_ray - start] = {
        .org = {origin.x(), origin.y(), origin.z()},
        .dir = {ray.x(), ray.y(), ray.z()},
        .t = 100,
        .hit = 0,
        .u = 0,
        .v = 0,
        .visibility = 0,
    };
  }

  std::vector<float> lux_values = trace_batch(pathtrace_tree, rays, 4);

  for (int i_ray = start; i_ray < end; i_ray++) {
    const Vector2i screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};

    const float r = std::min(lux_values[i_ray - start] * 255.0f, 255.0f);

    const Color new_color{
        .r = (uint8_t)(r),
        .g = (uint8_t)0,
        .b = (uint8_t)0,
        .a = (uint8_t)255,
    };

    Color interpolate_color = GetImageColor(target_image, screen_coords.x(), screen_coords.y());
    interpolate_color.r += (new_color.r - interpolate_color.r) * (reset ? 1.0f : 0.1f);

    ImageDrawPixel(&target_image, screen_coords.x(), screen_coords.y(), interpolate_color);
  }
}

void Scene::trace_image(BS::thread_pool &thread_pool, const Camera3D &camera, Image &target_image,
                        const Vector2i &pathtrace_area, const bool reset) {
  assert(IsImageReady(target_image));

  const Matrix4f viewMatrix = lookAt(tr(camera.position), tr(camera.target), tr(camera.up));
  const Matrix4f projectionMatrix =
      perspective<float>(camera.fovy, ((float)pathtrace_area.x() / (float)pathtrace_area.y()), RL_CULL_DISTANCE_NEAR,
                         RL_CULL_DISTANCE_FAR);

  const Matrix4f inv_view_proj = (projectionMatrix * viewMatrix).inverse();
  const Vector3f origin = tr(camera.position);

  const auto trace_task = [&](const int32_t start, const int32_t end) {
    first_trace(pathtrace_area, inv_view_proj, origin, start, end, target_image, reset);
  };

  const int32_t ray_count = pathtrace_area.x() * pathtrace_area.y();

  thread_pool.detach_blocks<int32_t>(0, ray_count, trace_task);
  thread_pool.wait();
}