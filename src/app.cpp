#include "app.hpp"

App::App() {
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
  if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON) || IsMouseButtonDown(MOUSE_MIDDLE_BUTTON) || GetMouseWheelMove() != 0.0f) {
    UpdateCamera(&scene.camera, CAMERA_FREE);

    gfx_context.pathtrace_steps = 1;
  }
}

void App::draw_viewport() {
  BeginMode3D(scene.camera);

  DrawGrid(10, 1.0f);

  // for (const auto [entity, transform, mesh] : scene.view<const Eigen::Isometry3f, const Mesh>().each()) {
  //   DrawMesh(mesh, default_material, Eigen::tr(transform.matrix()));
  // }

  // for (const auto [entity, transform, model] : scene.view<const Eigen::Isometry3f, const Model>().each()) {
  //   DrawModel(model,
  //             Vector3{transform.translation().x(), transform.translation().y(), transform.translation().z()},
  //             1.0f, WHITE);
  // }

  EndMode3D();
}

void App::draw_pathtrace() {
  pathtracer.rebuild_tree(scene);
  pathtracer.trace_image(thread_pool, scene.camera, gfx_context.pathtrace_area, gfx_context.pathtrace_steps,
                         gfx_context.pathtrace_image);

  gfx_context.pathtrace_steps++;

  Color *pathtrace_image_colors = LoadImageColors(gfx_context.pathtrace_image);
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
