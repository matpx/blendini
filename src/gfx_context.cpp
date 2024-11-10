#include "gfx_context.hpp"

#include <raylib.h>

#include <cassert>
#include <raylib-cpp.hpp>

#include "math_helper.hpp"

GfxContext::GfxContext() : window(1920, 1080, "Blendini", FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT) {
  window.SetTargetFPS(60);

  image_swap_pair->write_image =
      raylib::Image(next_power_of_two(window.GetWidth()), next_power_of_two(window.GetHeight()), BLACK);
  image_swap_pair->read_image = raylib::Image(image_swap_pair->write_image);

  pathtrace_texture.Load(image_swap_pair->read_image);

  rlImGuiSetup(true);
}

GfxContext::~GfxContext() { rlImGuiShutdown(); }