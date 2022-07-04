#pragma once
#include <Eigen/Geometry>
#include <vector>
class Hasher {
public:
  Hasher() {}
  template <class T> inline void hash_combine(std::size_t &s, const T &v) const{
    std::hash<T> h;
    s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
  };

  void CombineHashPose(std::size_t &hash, const Eigen::Affine3d &pose) const {
    const auto matrix = 1000.0 * pose.matrix();
    for (unsigned int i = 0; i < 3; ++i) {
      for (unsigned int j = i; j < 4; ++j) {
        hash_combine<double>(hash, trunc(matrix(i, j)));
      }
    }
  }
  template <class T> inline void CombineHashVector(std::size_t &s, const std::vector<T> &vector) const{
    for (const auto v : vector) {
      hash_combine<T>(s, v);
    }
  };

  // void CombineHashTranslation(std::size_t &hash, const Eigen::Vector3d &translation) {
  //   for (size_t i = 0; i < 3; i++) {
  //     hash_combine<double>(hash, trunc(translation(i)));
  //   }
  // }

  // void CombineHashRotation(std::size_t &hash, const Eigen::Matrix3d &matrix) {
  //   for (unsigned int i = 0; i < 3; ++i) {
  //     for (unsigned int j = i; j < 3; ++j) {
  //       hash_combine<double>(hash, trunc(matrix(i, j)));
  //     }
  //   }
  // }
};