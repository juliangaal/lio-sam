#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// #include "gtsamGravityFactor.hpp"
#include "gravity_factor/gravity_estimator.h"
#include "gravity_factor/gravity_factor.h"
#include "utility.h"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace lio_sam
{
class IMUPreintegration : public ParamServer
{
private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::mutex mtx;

  ros::Subscriber subImu;
  ros::Subscriber subOdometry;
  ros::Publisher  pubImuOdometry;

  bool systemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
  gtsam::Vector                           noiseModelBetweenBias;


  gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
  gtsam::PreintegratedImuMeasurements* imuIntegratorImu_;

  std::deque<sensor_msgs::Imu> imuQueOpt;
  std::deque<sensor_msgs::Imu> imuQueImu;

  gtsam::Pose3                 prevPose_;
  gtsam::Vector3               prevVel_;
  gtsam::NavState              prevState_;
  gtsam::imuBias::ConstantBias prevBias_;

  gtsam::NavState              prevStateOdom;
  gtsam::imuBias::ConstantBias prevBiasOdom;

  bool   doneFirstOpt = false;
  double lastImuT_imu = -1;
  double lastImuT_opt = -1;

  gtsam::ISAM2                optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values               graphValues;

  const double delta_t               = 0;
  double       currentCorrectionTime = -1.0f;
  int          key                   = 1;

  // T_bl: tramsform points from lidar frame to imu frame
  gtsam::Pose3 imu2Lidar = gtsam::Pose3( gtsam::Rot3( 1, 0, 0, 0 ), gtsam::Point3( -extTrans.x(), -extTrans.y(), -extTrans.z() ) );
  // T_lb: tramsform points from imu frame to lidar frame
  gtsam::Pose3 lidar2Imu = gtsam::Pose3( gtsam::Rot3( 1, 0, 0, 0 ), gtsam::Point3( extTrans.x(), extTrans.y(), extTrans.z() ) );

  // gravity optimization
  std::deque<Eigen::Vector3d>                   imuGravityVec;
  gtsam::noiseModel::Diagonal::shared_ptr       priorGravityNoise;
  Eigen::Vector3d                               gravityVec;          // always be ~(0,0,-9.8)
  Eigen::Vector3d                               gravityInBodyVec;    // in body(IMU) frame
  Eigen::Vector3d                               gravityInGlobalVec;  // in Global(ENU) frame
  GravityEstimator<double>                      gravityEstimator;
  std::deque<TransformAndPreintegrator<double>> transformAndPreintegratorQueue;
  std::deque<TransformAndPreintegrator<double>> transformAndPreintegratorQueueTemp;

public:
  IMUPreintegration();
  void resetOptimization();
  void resetParams();
  void trimOldIMUData();
  void odometryHandler( const nav_msgs::Odometry::ConstPtr& odomMsg );
  bool failureDetection( const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur );
  void imuHandler( const sensor_msgs::Imu::ConstPtr& imu_raw );
  bool estimateGravity();
};

}  // namespace lio_sam