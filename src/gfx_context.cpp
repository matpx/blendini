#include "gfx_context.hpp"

  GfxContext::GfxContext() {
    const int screenWidth = 1920;
    const int screenHeight = 1080;

    // SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Blendini");

    // SetTargetFPS(60);
    // EnableEventWaiting();

    rlImGuiSetup(true);

    pathtrace_area = {GetScreenWidth(), GetScreenHeight()};

    pathtrace_image =
        GenImageColor(next_power_of_two(pathtrace_area.x()), next_power_of_two(pathtrace_area.y()), BLACK);
    pathtrace_texture = LoadTextureFromImage(pathtrace_image);
  }

  GfxContext::~GfxContext() {
    UnloadTexture(pathtrace_texture);
    UnloadImage(pathtrace_image);

    rlImGuiShutdown();
    CloseWindow();
  }