#include <imgui/imgui.h>
#include <raylib.h>
#include <raymath.h>
#include <rjm/rjm_raytrace.h>
#include <rlImGui/rlImGui.h>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cassert>
#include <entt/entt.hpp>
#include <memory>
#include <span>
#include <vector>

struct Scene final : entt::registry {
  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;

  ~Scene() {
    if (tree.nodes != nullptr) {
      rjm_freeraytree(&tree);
    }

    for (const auto [entity, mesh] : view<Mesh>().each()) {
      if (mesh.vaoId > 0) {
        UnloadMesh(mesh);
      }
    }
  }

  RjmRayTree tree = {};

  std::vector<Eigen::Vector3f> vertices;
  std::vector<int32_t> indices;

  void rebuild() {
    if (tree.nodes != nullptr) {
      rjm_freeraytree(&tree);
    }

    vertices.clear();
    indices.clear();

    for (const auto [entity, transform, mesh] : view<Eigen::Isometry3f, Mesh>().each()) {
      const uint16_t vertex_end = vertices.size();

      static_assert(sizeof(Eigen::Vector3f) == sizeof(float) * 3);

      for (const auto &vertex :
           std::span<Eigen::Vector3f>(reinterpret_cast<Eigen::Vector3f *>(mesh.vertices), mesh.vertexCount)) {
        vertices.push_back(transform.translation() + transform.rotation() * vertex);
      }

      if (mesh.indices != nullptr) {
        for (const auto &index : std::span<uint16_t>(mesh.indices, mesh.triangleCount * 3)) {
          indices.push_back(vertex_end + index);
        }
      } else {
        for (int32_t i_index = 0; i_index < mesh.triangleCount * 3; i_index++) {
          indices.push_back(vertex_end + i_index);
        }
      }
    }

    tree.vtxs = reinterpret_cast<float *>(vertices.data());
    tree.tris = indices.data();
    tree.triCount = indices.size() / 3;

    rjm_buildraytree(&tree);
  }

  void trace_image(const Camera3D &camera, Image &target_image) {
    assert(IsImageReady(target_image));

    std::vector<RjmRay> rays(target_image.width * target_image.height);

    for (int x = 0; x < target_image.width; x++) {
      for (int y = 0; y < target_image.height; y++) {
        const Ray r = GetMouseRay(Vector2{static_cast<float>(x), static_cast<float>(y)}, camera);

        rays[x * target_image.width + y] = {
            .org = {r.position.x, r.position.y, r.position.z},
            .dir = {r.direction.x, r.direction.y, r.direction.z},
            .t = 100,
            .hit = 0,
            .u = 0,
            .v = 0,
            .visibility = 0,
        };
      }
    }

#if 0
    const uint32_t thread_count = std::thread::hardware_concurrency();
    std::vector<std::thread> render_threads;

    for (uint32_t i_thread = 0; i_thread < thread_count; i_thread++) {
      render_threads.push_back(std::thread(
          [&](const uint32_t thread_id) {
            rjm_raytrace(&tree, rays.size() / thread_count, rays.data() + thread_id * rays.size() / thread_count,
                         RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);
          },
          i_thread));
    }

    for (std::thread &thread : render_threads) {
      thread.join();
    }
#else
    rjm_raytrace(&tree, rays.size(), rays.data(), RJM_RAYTRACE_FIRSTHIT, nullptr, nullptr);
#endif

    for (int x = 0; x < target_image.width; x++) {
      for (int y = 0; y < target_image.height; y++) {
        const RjmRay r = rays[x * target_image.width + y];

        const Color c{
            .r = (uint8_t)255,
            .g = (uint8_t)(r.u * 255),
            .b = (uint8_t)(r.v * 255),
            .a = (uint8_t)255,
        };

        if (r.hit != -1) {
          ImageDrawPixel(&target_image, x, y, c);
        } else {
          ImageDrawPixel(&target_image, x, y, Color{0, 0, 0, 0});
        }
      }
    }
  }
};

int main(void) {
  const int screenWidth = 1024;
  const int screenHeight = 1024;

  SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
  InitWindow(screenWidth, screenHeight, "Blendini");

  SetTargetFPS(60);
  // EnableEventWaiting();

  rlImGuiSetup(true);

  {
    Camera3D camera = {
        .position = Vector3{10.0f, 10.0f, 10.0f},
        .target = Vector3{0.0f, 0.0f, 0.0f},
        .up = Vector3{0.0f, 1.0f, 0.0f},
        .fovy = 45.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    std::unique_ptr<Scene> scene = std::make_unique<Scene>();
    const entt::entity sphere_entity = scene->create();
    scene->emplace<Eigen::Isometry3f>(sphere_entity, Eigen::Isometry3f::Identity());
    scene->get<Eigen::Isometry3f>(sphere_entity).translate(Eigen::Vector3f{2, 0, 0});

    scene->emplace<Mesh>(sphere_entity, GenMeshSphere(1, 8, 8));
    scene->rebuild();

    Image pathtrace_image = GenImageColor(GetScreenWidth(), GetScreenHeight(), BLACK);
    Texture2D pathtrace_target = LoadTextureFromImage(pathtrace_image);

    Material default_material = LoadMaterialDefault();

    while (!WindowShouldClose()) {
      UpdateCamera(&camera, CAMERA_ORBITAL);

      BeginDrawing();

      ClearBackground(GRAY);

      {
        BeginMode3D(camera);

        DrawGrid(10, 1.0f);

        for (const auto [entity, transform, mesh] : scene->view<Eigen::Isometry3f, Mesh>().each()) {
          Eigen::Matrix4f m = transform.matrix().transpose();
          const Matrix *rl_world = reinterpret_cast<const Matrix *>(m.data());
          DrawMesh(mesh, default_material, *rl_world);
        }

        EndMode3D();
      }

      {
        scene->trace_image(camera, pathtrace_image);

        Color *pathtrace_image_colors = LoadImageColors(pathtrace_image);
        UpdateTexture(pathtrace_target, pathtrace_image_colors);
        UnloadImageColors(pathtrace_image_colors);

        DrawTexture(pathtrace_target, 0, 0, {255, 255, 255, 255});
      }

      {
        rlImGuiBegin();

        // ImGui::Begin("My First Tool", nullptr, ImGuiWindowFlags_MenuBar);
        // ImGui::End();

        rlImGuiEnd();
      }

      EndDrawing();
    }

    UnloadTexture(pathtrace_target);
    UnloadImage(pathtrace_image);
    UnloadMaterial(default_material);
  }

  rlImGuiShutdown();
  CloseWindow();

  return 0;
}
