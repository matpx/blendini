#include "gfx_context.hpp"

#include <raylib.h>

#include "math_helper.hpp"

GfxContext::GfxContext() {
  const int screenWidth = 1920;
  const int screenHeight = 1080;

  SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
  InitWindow(screenWidth, screenHeight, "Blendini");

  SetTargetFPS(60);
  // EnableEventWaiting();

  rlImGuiSetup(true);

  pathtrace_area = {GetScreenWidth(), GetScreenHeight()};
  image_swap_pair->write_image =
      GenImageColor(next_power_of_two(pathtrace_area.x()), next_power_of_two(pathtrace_area.y()), BLACK);
  image_swap_pair->read_image =
      GenImageColor(next_power_of_two(pathtrace_area.x()), next_power_of_two(pathtrace_area.y()), BLACK);
  pathtrace_texture = LoadTextureFromImage(image_swap_pair->read_image);

  default_shader =
      LoadShader(TextFormat("src/extern/raylib/examples/shaders/resources/shaders/glsl%i/lighting.vs", GLSL_VERSION),
                 TextFormat("src/extern/raylib/examples/shaders/resources/shaders/glsl%i/lighting.fs", GLSL_VERSION));

  assert(IsShaderReady(default_shader));  // TODO

  default_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(default_shader, "viewPos");
  default_shader.locs[SHADER_LOC_COLOR_AMBIENT] = GetShaderLocation(default_shader, "ambient");

  default_material = LoadMaterialDefault();
  default_material.shader = default_shader;
}

GfxContext::~GfxContext() {
  UnloadMaterial(default_material);

  UnloadTexture(pathtrace_texture);
  UnloadImage(image_swap_pair->read_image);
  UnloadImage(image_swap_pair->write_image);

  rlImGuiShutdown();
  CloseWindow();
}