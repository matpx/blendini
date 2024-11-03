#include "scene.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <BS_thread_pool.hpp>

#include "raymath_eigen.hpp"

using namespace Eigen;

// const auto set_image_pixel = [](Image &image, const Vector2i &pos, const Color &color) {
//   assert(IsImageReady(image));
//   assert((image.data != NULL) && (pos.x() >= 0) && (pos.x() < image.width) && (pos.y() >= 0) &&
//          (pos.y() < image.height));

//   *(reinterpret_cast<Color *>(image.data) + (pos.y() * image.width + pos.x())) = color;
// };

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

std::pair<Vector3f, Vector3f> Scene::get_diffuse_ray(const RjmRay &output_ray) const {
  const std::array<int32_t, 3> triangle_indices = {pathtrace_indices[output_ray.hit * 3],
                                                   pathtrace_indices[output_ray.hit * 3 + 1],
                                                   pathtrace_indices[output_ray.hit * 3 + 2]};

  const std::array<Vector3f, 3> triangle_vertices = {pathtrace_vertices[triangle_indices[0]],
                                                     pathtrace_vertices[triangle_indices[1]],
                                                     pathtrace_vertices[triangle_indices[2]]};

  const Vector3f normal =
      (triangle_vertices[1] - triangle_vertices[0]).cross(triangle_vertices[2] - triangle_vertices[0]);

  const Vector3f origin = (1.0f - output_ray.u - output_ray.v) * triangle_vertices[0] +
                          output_ray.u * triangle_vertices[1] + output_ray.v * triangle_vertices[2];

  const Vector3f offset_origin = origin + normal * 0.01f;

  return {offset_origin, normal};
}

float Scene::diffuse_trace(const RjmRay &input_ray, const int32_t ray_count, const int32_t steps_left) const {
  if (input_ray.hit == -1) {
    const auto get_sun = []() -> float { return 0.8f; };

    return get_sun();
  }

  if (steps_left <= 0) {
    return 0.0;
  }

  const auto [input_ray_origin, input_ray_normal] = get_diffuse_ray(input_ray);

  std::vector<RjmRay> diffuse_rays(ray_count);

  Vector3f diffuse_dir = Vector3f::Random();

  if (diffuse_dir.dot(input_ray_normal) < 0) {
    diffuse_dir *= -1;
  }

  for (int32_t i_diffuse_ray = 0; i_diffuse_ray < ray_count; i_diffuse_ray++) {
    diffuse_rays[i_diffuse_ray] = {
        .org = {input_ray_origin.x(), input_ray_origin.y(), input_ray_origin.z()},
        .dir = {diffuse_dir.x(), diffuse_dir.y(), diffuse_dir.z()},
        .t = 100,
        .hit = 0,
        .u = 0,
        .v = 0,
        .visibility = 0,
    };
  }

  rjm_raytrace(&pathtrace_tree, diffuse_rays.size(), diffuse_rays.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);

  const int32_t next_ray_count = ray_count / 2;
  float lux = 0.0f;

  for (const RjmRay &diffuse_ray : diffuse_rays) {
    lux += diffuse_trace(diffuse_ray, next_ray_count / 2, steps_left - 1) / diffuse_rays.size();
  }

  return lux;
};

void Scene::first_trace(const Vector2i &pathtrace_area, const Matrix4f &inv_view_proj, const Vector3f &origin,
                        const int32_t start, const int32_t end, Image &target_image) {
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

  rjm_raytrace(&pathtrace_tree, rays.size(), rays.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);

  for (int i_ray = start; i_ray < end; i_ray++) {
    const Vector2i screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};
    const RjmRay &output_ray = rays[i_ray - start];

    if (output_ray.hit != -1) {
      float l = diffuse_trace(output_ray, 16, 2);

      const Color c{
          .r = (uint8_t)(l * 255.0f),
          .g = (uint8_t)0,
          .b = (uint8_t)0,
          .a = (uint8_t)255,
      };

      ImageDrawPixel(&target_image, screen_coords.x(), screen_coords.y(), c);
    } else {
      ImageDrawPixel(&target_image, screen_coords.x(), screen_coords.y(), Color{0, 0, 0, 0});
    }
  }
}

void Scene::trace_image(BS::thread_pool &thread_pool, const Camera3D &camera, Image &target_image,
                        const Vector2i &pathtrace_area) {
  assert(IsImageReady(target_image));

  const Matrix4f viewMatrix = lookAt(tr(camera.position), tr(camera.target), tr(camera.up));
  const Matrix4f projectionMatrix =
      perspective<float>(camera.fovy, ((float)pathtrace_area.x() / (float)pathtrace_area.y()), RL_CULL_DISTANCE_NEAR,
                         RL_CULL_DISTANCE_FAR);

  const Matrix4f inv_view_proj = (projectionMatrix * viewMatrix).inverse();
  const Vector3f origin = tr(camera.position);

  const auto trace_task = [&](const int32_t start, const int32_t end) {
    first_trace(pathtrace_area, inv_view_proj, origin, start, end, target_image);
  };

  const int32_t ray_count = pathtrace_area.x() * pathtrace_area.y();

  thread_pool.detach_blocks<int32_t>(0, ray_count, trace_task);
  thread_pool.wait();
}