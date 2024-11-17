#pragma once

#include <BS_thread_pool.hpp>

#include "gfx_context.hpp"
#include "pathtracer.hpp"
#include "scene.hpp"

class App {
 public:
  enum class Mode {
    VIEWPORT,
    PATHTRACE,
  };

 private:
  Mode current_mode = Mode::VIEWPORT;

 public:
  std::shared_ptr<BS::thread_pool> thread_pool;

  GfxContext gfx_context;

  Scene scene;
  Pathtracer pathtracer;

  std::shared_ptr<raylib::Model> monkey = std::make_shared<raylib::Model>("monkey.glb");

  bool user_input_occured = true;

  App(const App &) = delete;
  App(App &&) = delete;

  App();

  void process_inputs();
  void draw_viewport();
  void draw_pathtrace();
  void draw_viewport_ui();
  void update();
  void run();
};