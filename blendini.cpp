#include <imgui.h>
#include <raylib.h>
#include <rjm_raytrace.h>
#include <rlImGui.h>
#include <stdint.h>

#include <vector>

struct Scene {
  Scene() = default;
  Scene(const Scene &) = delete;
  Scene(Scene &&) = delete;

  std::vector<Mesh> meshes;
  RjmRayTree tree = {};

  std::vector<float> vertices;
  std::vector<float> indices;

  void rebuild() {
    vertices.clear();
    indices.clear();

    for (const Mesh &mesh : meshes) {
      const size_t vertex_end = vertices.size();

      for (int i_vertex = 0; i_vertex < mesh.vertexCount; i_vertex++) {
        vertices.push_back(mesh.vertices[i_vertex]);
      }

      for (int i_index = 0; i_index < mesh.triangleCount * 3; i_index++) {
        indices.push_back(vertex_end + mesh.indices[i_index]);
      }
    }

    rjm_buildraytree(&tree);
  }

  void free() {
    rjm_freeraytree(&tree);

    for (const Mesh &mesh : meshes) {
      UnloadMesh(mesh);
    }
  }
};

constexpr static void set_pixel(Image &dst, const int32_t x, const int32_t y, const Color &color) {
  assert(dst.format == PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);

  ((unsigned char *)dst.data)[(y * dst.width + x) * 4] = color.r;
  ((unsigned char *)dst.data)[(y * dst.width + x) * 4 + 1] = color.g;
  ((unsigned char *)dst.data)[(y * dst.width + x) * 4 + 2] = color.b;
  ((unsigned char *)dst.data)[(y * dst.width + x) * 4 + 3] = color.a;
};

void trace_image(Image &target_image) {
  assert(IsImageReady(target_image));
  assert(target_image.format == PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);

  for (int x = 0; x < target_image.width; x++) {
    for (int y = 0; y < target_image.width; y++) {
      {
        const Color c{
            .r = (uint8_t)(x / 4 % 256),
            .g = (uint8_t)(y / 4 % 256),
            .b = (uint8_t)(y / 4 % 256),
            .a = (uint8_t)255,
        };

        set_pixel(target_image, x, y, c);
      }
    }
  }
}

int main(void) {
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
  InitWindow(screenWidth, screenHeight, "Blendini");

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

  Scene scene;
  scene.meshes.push_back(GenMeshCube(1, 1, 1));
  scene.rebuild();

  Image it = GenImageColor(1024, 1024, RED);
  trace_image(it);

  const Texture2D t = LoadTextureFromImage(it);
  UnloadImage(it);

  while (!WindowShouldClose()) {
    UpdateCamera(&camera, CAMERA_PERSPECTIVE);

    BeginDrawing();

    ClearBackground(RAYWHITE);

    {
      BeginMode3D(camera);

      DrawGrid(10, 1.0f);

      EndMode3D();
    }

    { DrawTexture(t, 0, 0, WHITE); }

    {
      rlImGuiBegin();

      // ImGui::Begin("My First Tool", nullptr, ImGuiWindowFlags_MenuBar);
      // ImGui::End();

      rlImGuiEnd();
    }

    EndDrawing();
  }

  UnloadTexture(t);

  rlImGuiShutdown();
  CloseWindow();

  return 0;
}
