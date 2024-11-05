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
  BS::thread_pool thread_pool;

  GfxContext gfx_context;

  Scene scene;
  Pathtracer pathtracer;

  Model monkey;

  App(const App &) = delete;
  App(App &&) = delete;

  App();
  ~App();

  void process_inputs();
  void draw_viewport();
  void draw_pathtrace();
  void draw_viewport_ui();
  void update();
  void run();
};