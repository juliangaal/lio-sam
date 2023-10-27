#pragma once
#include <glog/logging.h>

#include <deque>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "dataType.hpp"

namespace lio_sam
{
template <typename T>
class GravityEstimator
{
public:
  explicit GravityEstimator() {}
  bool Estimate(
      const std::deque<TransformAndPreintegrator<T>> &all_laser_transforms,
      const Eigen::Transform<T, 3, Eigen::Affine> &   transform_lb,
      const std::deque<Eigen::Matrix<T, 3, 1>> &      Vs,
      T                                               g_norm,
      Eigen::Matrix<T, 3, 1> &                        g_approx );

private:
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> TangentBasis( Eigen::Matrix<T, 3, 1> &g0 );

  bool ApproximateGravity(
      const std::deque<TransformAndPreintegrator<T>> &all_laser_transforms,
      const Eigen::Transform<T, 3, Eigen::Affine> &   transform_lb,
      const std::deque<Eigen::Matrix<T, 3, 1>> &      Vs,
      const T gnorm, Eigen::Matrix<T, 3, 1> &g );

  void RefineGravity(
      const std::deque<TransformAndPreintegrator<T>> &all_laser_transforms,
      const Eigen::Transform<T, 3, Eigen::Affine> &   transform_lb,
      const std::deque<Eigen::Matrix<T, 3, 1>> &      Vs,
      const T                                         gnorm,
      Eigen::Matrix<T, 3, 1> &                        g_approx );

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}  // namespace lio_sam
