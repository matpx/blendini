#include "scene.hpp"

#include <BS_thread_pool.hpp>

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

  for (const auto [entity, transform, mesh] : view<Eigen::Isometry3f, Mesh>().each()) {
    const uint16_t last_vertex = pathtrace_vertices.size();

    static_assert(sizeof(Eigen::Vector3f) == sizeof(float) * 3);

    for (const auto &vertex :
         std::span<Eigen::Vector3f>(reinterpret_cast<Eigen::Vector3f *>(mesh.vertices), mesh.vertexCount)) {
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
                        const Eigen::Vector2i &pathtrace_area) {
  assert(IsImageReady(target_image));

  std::vector<RjmRay> rays(pathtrace_area.x() * pathtrace_area.y());

  for (int x = 0; x < pathtrace_area.x(); x++) {
    for (int y = 0; y < pathtrace_area.y(); y++) {
      const Ray camera_ray = GetMouseRay(Vector2{static_cast<float>(x), static_cast<float>(y)}, camera);

      rays[x * pathtrace_area.y() + y] = {
          .org = {camera_ray.position.x, camera_ray.position.y, camera_ray.position.z},
          .dir = {camera_ray.direction.x, camera_ray.direction.y, camera_ray.direction.z},
          .t = 100,
          .hit = 0,
          .u = 0,
          .v = 0,
          .visibility = 0,
      };
    }
  }

#if 1
  const auto task = [&](const int32_t start, const int32_t end) {
    rjm_raytrace(&pathtrace_tree, end - start, rays.data() + start, RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);
  };

  thread_pool.detach_blocks<int32_t>(0, rays.size(), task);
  thread_pool.wait();
#else
  rjm_raytrace(&pathtrace_tree, rays.size(), rays.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);
#endif

  //   const auto set_pixel = [](Image &target, const int32_t x, const int32_t y, const Color &color) {
  //     assert(target.format == PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
  //     if ((target.data == NULL) || (x < 0) || (x >= target.width) || (y < 0) || (y >= target.height)) return;

  //     ((unsigned char *)target.data)[(y * target.width + x) * 4] = color.r;
  //     ((unsigned char *)target.data)[(y * target.width + x) * 4 + 1] = color.g;
  //     ((unsigned char *)target.data)[(y * target.width + x) * 4 + 2] = color.b;
  //     ((unsigned char *)target.data)[(y * target.width + x) * 4 + 3] = color.a;
  //   };

  for (int x = 0; x < pathtrace_area.x(); x++) {
    for (int y = 0; y < pathtrace_area.y(); y++) {
      const RjmRay output_ray = rays[x * pathtrace_area.y() + y];

      const Color c{
          .r = (uint8_t)255,
          .g = (uint8_t)(output_ray.u * 255),
          .b = (uint8_t)(output_ray.v * 255),
          .a = (uint8_t)255,
      };

      if (output_ray.hit != -1) {
        ImageDrawPixel(&target_image, x, y, c);
      } else {
        ImageDrawPixel(&target_image, x, y, Color{0, 0, 0, 0});
      }
    }
  }
}