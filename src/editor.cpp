#include "editor.hpp"

#include <imgui.h>

namespace editor {

void draw() {
  ImGui::Begin("My First Tool", nullptr, 0);
  ImGui::Button("hi");
  ImGui::End();

  ImGui::Begin("My First Tool2", nullptr, 0);
  ImGui::Button("hi2");
  ImGui::End();
}

}  // namespace editor