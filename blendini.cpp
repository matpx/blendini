#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

int main(void) {
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
  InitWindow(screenWidth, screenHeight,
             "Blendini");

  SetTargetFPS(60);
  // EnableEventWaiting();

  rlImGuiSetup(true);

  Camera3D camera = {
      .position = Vector3{10.0f, 10.0f, 10.0f},
      .target = Vector3{0.0f, 0.0f, 0.0f},
      .up = Vector3{0.0f, 1.0f, 0.0f},
      .fovy = 45.0f,
      .projection = CAMERA_PERSPECTIVE,
  };

  const Vector3 cubePosition = {0.0f, 0.0f, 0.0f};

  while (!WindowShouldClose()) {
    UpdateCamera(&camera, CAMERA_PERSPECTIVE);

    BeginDrawing();

    ClearBackground(RAYWHITE);

    BeginMode3D(camera);

    DrawCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
    DrawCubeWires(cubePosition, 2.0f, 2.0f, 2.0f, MAROON);

    DrawGrid(10, 1.0f);

    EndMode3D();

    rlImGuiBegin();

    ImGui::Begin("My First Tool", nullptr, ImGuiWindowFlags_MenuBar);
    ImGui::End();

    rlImGuiEnd();

    EndDrawing();
  }

  rlImGuiShutdown();
  CloseWindow();

  return 0;
}
