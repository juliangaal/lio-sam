#pragma once

#include "utility.h"

namespace lio_sam
{
class TransformFusion : public ParamServer
{
private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  std::mutex mtx;

  ros::Subscriber subImuOdometry;
  ros::Subscriber subLaserOdometry;

  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  Eigen::Affine3f lidarOdomAffine;
  Eigen::Affine3f imuOdomAffineFront;
  Eigen::Affine3f imuOdomAffineBack;

  tf::TransformListener tfListener;
  tf::StampedTransform  lidar2Baselink;

  double lidarOdomTime = -1;

  std::deque<nav_msgs::Odometry> imuOdomQueue;

public:
  TransformFusion();
  void lidarOdometryHandler( const nav_msgs::Odometry::ConstPtr& odomMsg );
  void imuOdometryHandler( const nav_msgs::Odometry::ConstPtr& odomMsg );
};
}  // namespace lio_sam