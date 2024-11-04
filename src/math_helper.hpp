#pragma once

#include <Eigen/Dense>

auto next_power_of_two(auto value) -> decltype(value) {
  decltype(value) power = 1;

  while (power < value) {
    power *= 2;
  }

  return power;
}

[[nodiscard]]
inline Eigen::Vector3f screen_to_world(const Eigen::Vector2f& screen_pos, const Eigen::Vector2i& screen_size,
                                       const Eigen::Matrix4f& inv_view_proj) {
  const Eigen::Vector2f ndc{
      (2.0f * screen_pos.x()) / screen_size.x() - 1.0f,
      1.0f - (2.0f * screen_pos.y()) / screen_size.y(),
  };

  const Eigen::Vector4f direction = inv_view_proj * Eigen::Vector4f(ndc.x(), ndc.y(), 1.0f, 1.0f);

  return direction.head<3>().normalized();
}