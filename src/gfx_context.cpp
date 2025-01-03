#include "gfx_context.hpp"

#include <imgui.h>
#include <rlImGui/rlImGui.h>

GfxContext::GfxContext()
    : window(1920, 1080, "Blendini", FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT),
      image_swap_pair(std::make_shared<ImageSwapPair>(Eigen::Vector2i{window.GetWidth(), window.GetHeight()})) {
  window.SetTargetFPS(60);

  rlImGuiSetup(true);
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

GfxContext::~GfxContext() { rlImGuiShutdown(); }