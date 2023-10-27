#pragma once
#include <gtsam/navigation/ImuFactor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// for gravity optimization
template <typename T>
class TransformAndPreintegrator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Transform<T, 3, Eigen::Affine>                transform;
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> pre_integration;

  explicit TransformAndPreintegrator()
  {
    transform       = Eigen::Transform<T, 3, Eigen::Affine>::Identity();
    pre_integration = nullptr;
  }

  TransformAndPreintegrator( const Eigen::Transform<T, 3, Eigen::Affine>& transform_, const std::shared_ptr<gtsam::PreintegratedImuMeasurements>& imu_integrator_ )
      : transform( transform_ ), pre_integration( imu_integrator_ )
  {
  }
};

enum class SensorType
{
  VELODYNE,
  LEISHEN,
  OUSTER,
  LIVOX
};

struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float    time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT( VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)( float, time, time ) )

struct OusterPointXYZIRT
{
  PCL_ADD_POINT4D;
  float    intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t  ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT( OusterPointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, noise, noise)( std::uint32_t, range, range ) )

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
  float  roll;
  float  pitch;
  float  yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIRPYT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)( double, time, time ) )


// Use the Velodyne point format as a common representation
using PointType     = pcl::PointXYZI;
using PointXYZIRT   = VelodynePointXYZIRT;
using PointTypePose = PointXYZIRPYT;