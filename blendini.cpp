#include <rlImGui/rlImGui.h>

#include <entt/entt.hpp>

#include "raymath_eigen.hpp"
#include "scene.hpp"

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
    scene->emplace<Mesh>(sphere_entity, GenMeshSphere(1, 8, 8));
    scene->emplace<Eigen::Isometry3f>(sphere_entity, Eigen::Isometry3f::Identity());
    scene->get<Eigen::Isometry3f>(sphere_entity).translate(Eigen::Vector3f{2, 0, 0});

    scene->rebuild();

    Image pathtrace_image = GenImageColor(GetScreenWidth(), GetScreenHeight(), BLACK);
    Texture2D pathtrace_texture = LoadTextureFromImage(pathtrace_image);

    Material default_material = LoadMaterialDefault();

    while (!WindowShouldClose()) {
      UpdateCamera(&camera, CAMERA_ORBITAL);

      BeginDrawing();

      ClearBackground(GRAY);

      {
        BeginMode3D(camera);

        DrawGrid(10, 1.0f);

        for (const auto [entity, transform, mesh] : scene->view<Eigen::Isometry3f, Mesh>().each()) {
          DrawMesh(mesh, default_material, to(transform.matrix()));
        }

        EndMode3D();
      }

      {
        scene->trace_image(camera, pathtrace_image);

        Color *pathtrace_image_colors = LoadImageColors(pathtrace_image);
        UpdateTexture(pathtrace_texture, pathtrace_image_colors);
        UnloadImageColors(pathtrace_image_colors);

        DrawTexture(pathtrace_texture, 0, 0, {255, 255, 255, 255});
      }

      {
        rlImGuiBegin();

        // ImGui::Begin("My First Tool", nullptr, ImGuiWindowFlags_MenuBar);
        // ImGui::End();

        rlImGuiEnd();
      }

      EndDrawing();
    }

    UnloadTexture(pathtrace_texture);
    UnloadImage(pathtrace_image);
    UnloadMaterial(default_material);
  }

  rlImGuiShutdown();
  CloseWindow();

  return 0;
}
