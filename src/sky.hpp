#pragma once

#include <Eigen/Dense>

struct Sky {
  Eigen::Vector4f top_color = {0.5f, 0.7f, 1.0f, 1.0f};
  Eigen::Vector4f bottom_color = {1.0f, 1.0f, 1.0f, 1.0f};
};