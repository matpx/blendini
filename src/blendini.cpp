#include "app.hpp"

int main(void) {
  std::unique_ptr<App> app = std::make_unique<App>();

  app->run();

  return 0;
}
