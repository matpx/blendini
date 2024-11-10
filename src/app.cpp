#include "app.hpp"
#include <imgui.h>

#include <Keyboard.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <Mouse.hpp>
#include <memory>
#include <mutex>
#include <raylib-cpp.hpp>

#include "pathtracer.hpp"
#include "raymath_helper.hpp"

App::App() : pathtracer(gfx_context.image_swap_pair) {
  const entt::entity sphere_entity = scene.create();
  scene.emplace<std::shared_ptr<raylib::Mesh>>(sphere_entity,
                                               std::make_shared<raylib::Mesh>(raylib::Mesh::Sphere(1, 8, 8)));
  scene.emplace<Eigen::Isometry3f>(sphere_entity, Eigen::Isometry3f::Identity());
  scene.get<Eigen::Isometry3f>(sphere_entity).translate(Eigen::Vector3f{3, 1, 0});

  const entt::entity floor_entity = scene.create();
  scene.emplace<std::shared_ptr<raylib::Mesh>>(floor_entity,
                                               std::make_shared<raylib::Mesh>(raylib::Mesh::Plane(10, 10, 10, 10)));
  scene.emplace<Eigen::Isometry3f>(floor_entity, Eigen::Isometry3f::Identity());

  const entt::entity monkey_entity = scene.create();
  scene.emplace<std::shared_ptr<raylib::Model>>(monkey_entity, monkey);
  scene.emplace<Eigen::Isometry3f>(monkey_entity, Eigen::Isometry3f::Identity());
  scene.get<Eigen::Isometry3f>(monkey_entity).translate(Eigen::Vector3f{0, 1, 0});
}

void App::process_inputs() {
  user_input_occured = raylib::Mouse::IsButtonDown(MOUSE_RIGHT_BUTTON) ||
                       raylib::Mouse::IsButtonDown(MOUSE_MIDDLE_BUTTON) || raylib::Keyboard::IsKeyDown(KEY_F1) ||
                       raylib::Mouse::GetWheelMove() != 0.0f;

  if (user_input_occured) {
    scene.camera.Update(CAMERA_FREE);
  }

  if (raylib::Keyboard::IsKeyPressed(KEY_F1)) {
    current_mode = current_mode == Mode::VIEWPORT ? Mode::PATHTRACE : Mode::VIEWPORT;
  }
}

void App::draw_viewport() {
  if (current_mode != Mode::VIEWPORT) {
    return;
  }

  scene.camera.BeginMode();

  DrawGrid(10, 1.0f);

  for (const auto [entity, transform, mesh] :
       scene.view<const Eigen::Isometry3f, const std::shared_ptr<raylib::Mesh>>().each()) {
    mesh->Draw(gfx_context.default_material, tr(transform.matrix()));
  }

  for (const auto [entity, transform, model] :
       scene.view<const Eigen::Isometry3f, const std::shared_ptr<raylib::Model>>().each()) {
    model->Draw(Vector3{transform.translation().x(), transform.translation().y(), transform.translation().z()}, 1.0f,
                WHITE);
  }

  scene.camera.EndMode();
}

void App::draw_pathtrace() {
  if (current_mode != Mode::PATHTRACE) {
    return;
  }

  if (user_input_occured || !pathtracer.is_ready()) {
    gfx_context.pathtrace_steps = 0;
    pathtracer.rebuild_tree(scene);
  }

  pathtracer.trace_image(thread_pool, scene.camera, {gfx_context.window.GetWidth(), gfx_context.window.GetHeight()},
                         gfx_context.pathtrace_steps);

  gfx_context.pathtrace_steps++;

  Color *pathtrace_image_colors = nullptr;
  {
    std::lock_guard lock(gfx_context.image_swap_pair->swap_mutex);
    pathtrace_image_colors = gfx_context.image_swap_pair->read_image.LoadColors();
  }

  gfx_context.pathtrace_texture.Update(pathtrace_image_colors);
  gfx_context.image_swap_pair->read_image.UnloadColors(pathtrace_image_colors);

  gfx_context.pathtrace_texture.Draw(0, 0, {255, 255, 255, 255});
}

void App::draw_viewport_ui() {
  rlImGuiBegin();

  // ImGui::Begin("My First Tool", nullptr, ImGuiWindowFlags_MenuBar);
  // ImGui::End();

  rlImGuiEnd();

  gfx_context.window.DrawFPS(10, 10);
}

void App::update() {
  process_inputs();

  gfx_context.window.BeginDrawing();
  gfx_context.window.ClearBackground(GRAY);

  draw_viewport();
  draw_pathtrace();
  draw_viewport_ui();

  gfx_context.window.EndDrawing();
}

void App::run() {
  while (!gfx_context.window.ShouldClose()) {
    update();
  }
}
