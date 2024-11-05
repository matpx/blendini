#include "app.hpp"

#include <raylib.h>

#include <mutex>

#include "pathtracer.hpp"
#include "raymath_helper.hpp"

App::App() : pathtracer(gfx_context.image_swap_pair) {
  const entt::entity sphere_entity = scene.create();
  scene.emplace<Mesh>(sphere_entity, GenMeshSphere(1, 8, 8));
  scene.emplace<Eigen::Isometry3f>(sphere_entity, Eigen::Isometry3f::Identity());
  scene.get<Eigen::Isometry3f>(sphere_entity).translate(Eigen::Vector3f{3, 1, 0});

  const entt::entity floor_entity = scene.create();
  scene.emplace<Mesh>(floor_entity, GenMeshPlane(10, 10, 10, 10));
  scene.emplace<Eigen::Isometry3f>(floor_entity, Eigen::Isometry3f::Identity());

  monkey = LoadModel("monkey.glb");
  const entt::entity monkey_entity = scene.create();
  scene.emplace<Model>(monkey_entity, monkey);
  scene.emplace<Eigen::Isometry3f>(monkey_entity, Eigen::Isometry3f::Identity());
  scene.get<Eigen::Isometry3f>(monkey_entity).translate(Eigen::Vector3f{0, 1, 0});
}

App::~App() { UnloadModel(monkey); }

void App::process_inputs() {
  user_input_occured = IsMouseButtonDown(MOUSE_RIGHT_BUTTON) || IsMouseButtonDown(MOUSE_MIDDLE_BUTTON) ||
                       IsKeyDown(KEY_F1) || GetMouseWheelMove() != 0.0f;

  if (user_input_occured) {
    UpdateCamera(&scene.camera, CAMERA_FREE);
  }

  if (IsKeyPressed(KEY_F1)) {
    current_mode = current_mode == Mode::VIEWPORT ? Mode::PATHTRACE : Mode::VIEWPORT;
  }
}

void App::draw_viewport() {
  if (current_mode != Mode::VIEWPORT) {
    return;
  }

  SetShaderValue(gfx_context.default_shader, gfx_context.default_shader.locs[SHADER_LOC_VECTOR_VIEW],
                 &scene.camera.position, SHADER_UNIFORM_VEC3);

  SetShaderValue(gfx_context.default_shader, gfx_context.default_shader.locs[SHADER_LOC_COLOR_AMBIENT],
                 scene.sky.top_color.data(), SHADER_UNIFORM_VEC4);

  BeginMode3D(scene.camera);

  DrawGrid(10, 1.0f);

  for (const auto [entity, transform, mesh] : scene.view<const Eigen::Isometry3f, const Mesh>().each()) {
    DrawMesh(mesh, gfx_context.default_material, tr(transform.matrix()));
  }

  for (const auto [entity, transform, model] : scene.view<const Eigen::Isometry3f, const Model>().each()) {
    model.materials[0].shader = gfx_context.default_shader;
    DrawModel(model, Vector3{transform.translation().x(), transform.translation().y(), transform.translation().z()},
              1.0f, WHITE);
  }

  EndMode3D();
}

void App::draw_pathtrace() {
  if (current_mode != Mode::PATHTRACE) {
    return;
  }

  if (user_input_occured || !pathtracer.is_ready()) {
    gfx_context.pathtrace_steps = 0;
    pathtracer.rebuild_tree(scene);
  }

  pathtracer.trace_image(thread_pool, scene.camera, gfx_context.pathtrace_area, gfx_context.pathtrace_steps);

  gfx_context.pathtrace_steps++;

  gfx_context.image_swap_pair->swap_mutex.lock();
  Color *pathtrace_image_colors = LoadImageColors(gfx_context.image_swap_pair->read_image);
  gfx_context.image_swap_pair->swap_mutex.unlock();

  UpdateTexture(gfx_context.pathtrace_texture, pathtrace_image_colors);
  UnloadImageColors(pathtrace_image_colors);

  DrawTexture(gfx_context.pathtrace_texture, 0, 0, {255, 255, 255, 255});
}

void App::draw_viewport_ui() {
  rlImGuiBegin();

  // ImGui::Begin("My First Tool", nullptr, ImGuiWindowFlags_MenuBar);
  // ImGui::End();

  rlImGuiEnd();

  DrawFPS(10, 10);
}

void App::update() {
  process_inputs();

  BeginDrawing();

  ClearBackground(GRAY);

  draw_viewport();
  draw_pathtrace();
  draw_viewport_ui();

  EndDrawing();
}

void App::run() {
  while (!WindowShouldClose()) {
    update();
  }
}
