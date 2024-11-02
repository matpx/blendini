#include "scene.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <BS_thread_pool.hpp>

#include "raymath_eigen.hpp"

using namespace Eigen;

inline static Vector3f screen_to_world_ray(const Vector2f &screenPos, const Vector2i &screenSize,
                                           const Matrix4f &invViewProj) {
  // Convert screen position to normalized device coordinates (NDC)
  const Vector2f ndc{
      (2.0f * screenPos.x()) / screenSize.x() - 1.0f,
      1.0f - (2.0f * screenPos.y()) / screenSize.y(),
  };

  // NDC to homogeneous clip space (z = -1 for near plane and w = 1)
  const Vector4f nearPointNDC(ndc.x(), ndc.y(), -1.0f, 1.0f);
  const Vector4f farPointNDC(ndc.x(), ndc.y(), 1.0f, 1.0f);

  // Convert NDC points to world space
  Vector4f nearPointWorld = invViewProj * nearPointNDC;
  Vector4f farPointWorld = invViewProj * farPointNDC;

  // Divide by w to get actual 3D coordinates in world space
  nearPointWorld /= nearPointWorld.w();
  farPointWorld /= farPointWorld.w();

  // The ray origin is the camera position, which we can get from the view matrix
  // Assuming the camera is at the origin in view space
  return (farPointWorld.head<3>() - nearPointWorld.head<3>()).normalized();
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

  const Matrix4f invViewProj = (projectionMatrix * viewMatrix).inverse();
  const Vector3f rayOrigin = viewMatrix.inverse().block<3, 1>(0, 3);

  const auto task = [&](const int32_t start, const int32_t end) {
    assert(start < end);

    std::vector<RjmRay> rays(end - start);

    for (int i_ray = start; i_ray < end; i_ray++) {
      const Vector2f screen_coords{i_ray % pathtrace_area.x(), i_ray / pathtrace_area.x()};
      const Vector3f ray = screen_to_world_ray(screen_coords, pathtrace_area, invViewProj);

      rays[i_ray - start] = {
          .org = {rayOrigin.x(), rayOrigin.y(), rayOrigin.z()},
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