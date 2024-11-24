#include "app.hpp"

#include <rlImGui/rlImGui.h>

#include "editor.hpp"
#include "pathtracer.hpp"
#include "raymath_helper.hpp"

App::App() : thread_pool(std::make_shared<BS::thread_pool>()), pathtracer(thread_pool, gfx_context.image_swap_pair) {
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
    pathtracer.stop_and_join();
    current_mode = current_mode == Mode::VIEWPORT ? Mode::PATHTRACE : Mode::VIEWPORT;
  }
}

void App::draw_viewport() {
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

  const Pathtracer::Status pathtracer_status = pathtracer.get_status();

  if (user_input_occured) {
    pathtracer.request_stop();
  } else if (pathtracer_status == Pathtracer::Status::STOPPED || pathtracer_status == Pathtracer::Status::ABORT) {
    pathtracer.rebuild_tree(scene);
    pathtracer.start(scene.camera, gfx_context.image_swap_pair->size, max_pathtrace_step);
  } else {
    if (pathtracer_status != Pathtracer::Status::FINISHED) {
      gfx_context.image_swap_pair->update_texture();
    }

    gfx_context.image_swap_pair->pathtrace_texture.Draw(0, 0, {255, 255, 255, 255});

    raylib::DrawText(TextFormat("step: %d", pathtracer.get_current_step()), 10, 40, 20, raylib::Color::Black());

    if (pathtracer_status == Pathtracer::Status::FINISHED) {
      raylib::DrawText(TextFormat("finished!"), 10, 60, 20, raylib::Color::Black());
    }
  }
}

void App::draw_viewport_ui() {
  rlImGuiBegin();

  editor::draw();

  rlImGuiEnd();

  gfx_context.window.DrawFPS(10, 10);
}

void App::update() {
  process_inputs();

  gfx_context.window.BeginDrawing();
  gfx_context.window.ClearBackground(raylib::Color::Gray());

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
