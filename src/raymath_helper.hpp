#pragma once

#include <Eigen/Dense>
#include <raylib-cpp.hpp>

inline raylib::Matrix tr(const Eigen::Matrix4f& from) {
  const auto data = from.data();

  return raylib::Matrix{
      data[0], data[4], data[8],  data[12], data[1], data[5], data[9],  data[13],
      data[2], data[6], data[10], data[14], data[3], data[7], data[11], data[15],
  };
}

inline raylib::Vector3 tr(const Eigen::Vector3f& from) { return raylib::Vector3{from.x(), from.y(), from.z()}; }
inline Eigen::Vector3f tr(const Vector3& from) { return Eigen::Vector3f{from.x, from.y, from.z}; }
