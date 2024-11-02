#include "scene.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <BS_thread_pool.hpp>

#include "raymath_eigen.hpp"

using namespace Eigen;

inline static Vector3f screen_to_world(const Vector2f &screen_pos, const Vector2i &screen_size,
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

  for (const auto [entity, mesh] : view<Mesh>().each()) {
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

  for (const auto [entity, transform, mesh] : view<Isometry3f, Mesh>().each()) {
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

  pathtrace_tree.vtxs = reinterpret_cast<float *>(pathtrace_vertices.data());
  pathtrace_tree.tris = pathtrace_indices.data();
  pathtrace_tree.triCount = pathtrace_indices.size() / 3;

  rjm_buildraytree(&pathtrace_tree);
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

  const auto task = [&](const int32_t start, const int32_t end) {
    assert(start < end);

    std::vector<RjmRay> rays(end - start);

    for (int i_ray = start; i_ray < end; i_ray++) {
      const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};
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
      const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};
      const RjmRay &output_ray = rays[i_ray - start];

      const Color c{
          .r = (uint8_t)255,
          .g = (uint8_t)(output_ray.u * 255),
          .b = (uint8_t)(output_ray.v * 255),
          .a = (uint8_t)255,
      };

      if (output_ray.hit != -1) {
        ImageDrawPixel(&target_image, screen_coords.x(), screen_coords.y(), c);
      } else {
        ImageDrawPixel(&target_image, screen_coords.x(), screen_coords.y(), Color{0, 0, 0, 0});
      }
    }
  };

  thread_pool.detach_blocks<int32_t>(0, pathtrace_area.x() * pathtrace_area.y(), task);
  thread_pool.wait();
}