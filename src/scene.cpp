#include "scene.hpp"

Scene::~Scene() {
  for (const auto [entity, mesh] : view<const Mesh>().each()) {
    if (mesh.vaoId > 0) {
      UnloadMesh(mesh);
    }
  }
}
