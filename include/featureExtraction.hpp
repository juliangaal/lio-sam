#ifndef FEATURE_EXTRACTION_HPP
#define FEATURE_EXTRACTION_HPP

#include "lio_sam/cloud_info.h"
#include "utility/paramServer.hpp"
#include "utility/utility.h"

struct smoothness_t
{
  float  value;
  size_t ind;
};

struct by_value
{
  bool operator()( smoothness_t const &left, smoothness_t const &right )
  {
    return left.value < right.value;
  }
};

namespace lio_sam
{
class FeatureExtraction : public ParamServer
{
private:  // variables
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::Subscriber subLaserCloudInfo;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  lio_sam::cloud_info cloudInfo;
  std_msgs::Header    cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  float *                   cloudCurvature;
  int *                     cloudNeighborPicked;
  int *                     cloudLabel;

public:  // functions
  FeatureExtraction();
  ~FeatureExtraction();
  void initializationValue();
  void laserCloudInfoHandler( const lio_sam::cloud_infoConstPtr &msgIn );
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
  void freeCloudInfoMemory();
  void publishFeatureCloud();
};
}  // namespace lio_sam

#endif  // FEATURE_EXTRACTION_HPP