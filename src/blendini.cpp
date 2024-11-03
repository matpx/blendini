#include <rlImGui/rlImGui.h>

#include <BS_thread_pool.hpp>
#include <entt/entt.hpp>

#include "scene.hpp"

int main(void) {
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  BS::thread_pool thread_pool;

  // SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
  InitWindow(screenWidth, screenHeight, "Blendini");

  // SetTargetFPS(60);
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
    scene->get<Eigen::Isometry3f>(sphere_entity).translate(Eigen::Vector3f{2, 1, 0});

    const entt::entity floor_entity = scene->create();
    scene->emplace<Mesh>(floor_entity, GenMeshPlane(10, 10, 10, 10));
    scene->emplace<Eigen::Isometry3f>(floor_entity, Eigen::Isometry3f::Identity());

    // const Model lantern = LoadModel("Lantern.glb");
    // const entt::entity lantern_entity = scene->create();
    // scene->emplace<Model>(lantern_entity, lantern);
    // scene->emplace<Eigen::Isometry3f>(lantern_entity, Eigen::Isometry3f::Identity());

    const auto next_power_of_two = [](auto value) -> decltype(value) {
      decltype(value) power = 1;

      while (power < value) {
        power *= 2;
      }

      return power;
    };

    const Eigen::Vector2i pathtrace_area = {GetScreenWidth(), GetScreenHeight()};

    Image pathtrace_image =
        GenImageColor(next_power_of_two(pathtrace_area.x()), next_power_of_two(pathtrace_area.y()), BLACK);
    Texture2D pathtrace_texture = LoadTextureFromImage(pathtrace_image);

    Material default_material = LoadMaterialDefault();

    while (!WindowShouldClose()) {
      UpdateCamera(&camera, CAMERA_ORBITAL);

      BeginDrawing();

      ClearBackground(GRAY);

      {
        BeginMode3D(camera);

        DrawGrid(10, 1.0f);

        // for (const auto [entity, transform, mesh] : scene->view<const Eigen::Isometry3f, const Mesh>().each()) {
        //   DrawMesh(mesh, default_material, Eigen::tr(transform.matrix()));
        // }

        // for (const auto [entity, transform, model] : scene->view<const Eigen::Isometry3f, const Model>().each()) {
        //   DrawModel(model,
        //             Vector3{transform.translation().x(), transform.translation().y(), transform.translation().z()},
        //             1.0f, WHITE);
        // }

        EndMode3D();
      }

      {
        scene->rebuild();
        scene->trace_image(thread_pool, camera, pathtrace_image, pathtrace_area);

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

        DrawFPS(10, 10);
      }

      EndDrawing();
    }

    UnloadTexture(pathtrace_texture);
    UnloadImage(pathtrace_image);
    // UnloadModel(lantern);
    UnloadMaterial(default_material);
  }

  rlImGuiShutdown();
  CloseWindow();

  return 0;
}
