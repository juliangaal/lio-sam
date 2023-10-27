#ifndef IMAGE_PROJECTION_HPP
#define IMAGE_PROJECTION_HPP

#include "dataType.hpp"
#include "lio_sam/cloud_info.h"
#include "utility.h"

const int queueLength = 2000;

namespace lio_sam
{
class ImageProjection : public ParamServer
{
private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool   firstFlag;
  double timePrev, timeIncrement;

  std::mutex imuLock;
  std::mutex odoLock;

  ros::Subscriber subLaserCloud;
  ros::Publisher  pubLaserCloud;

  ros::Publisher pubExtractedCloud;
  ros::Publisher pubLaserCloudInfo;

  ros::Subscriber              subImu;
  std::deque<sensor_msgs::Imu> imuQueue;

  ros::Subscriber                subOdom;
  std::deque<nav_msgs::Odometry> odomQueue;

  std::deque<sensor_msgs::PointCloud2> cloudQueue;
  sensor_msgs::PointCloud2             currentCloudMsg;

  double *imuTime = new double[ queueLength ];
  double *imuRotX = new double[ queueLength ];
  double *imuRotY = new double[ queueLength ];
  double *imuRotZ = new double[ queueLength ];

  int             imuPointerCur;
  bool            firstPointFlag;
  Eigen::Affine3f transStartInverse;

  pcl::PointCloud<PointXYZIRT>::Ptr       laserCloudIn;
  pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
  pcl::PointCloud<PointType>::Ptr         fullCloud;
  pcl::PointCloud<PointType>::Ptr         extractedCloud;

  int     deskewFlag;
  cv::Mat rangeMat;

  bool  odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  lio_sam::cloud_info cloudInfo;
  double              timeScanCur;
  double              timeScanEnd;
  std_msgs::Header    cloudHeader;

  std::vector<int> columnIdnCountVec;

public:
  ImageProjection();
  ~ImageProjection();
  void      allocateMemory();
  void      resetParameters();
  void      imuHandler( const sensor_msgs::Imu::ConstPtr &imuMsg );
  void      odometryHandler( const nav_msgs::Odometry::ConstPtr &odomMsg );
  void      cloudHandler( const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg );
  bool      cachePointCloud( const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg );
  bool      deskewInfo();
  void      imuDeskewInfo();
  void      odomDeskewInfo();
  void      findRotation( double pointTime, float *rotXCur, float *rotYCur, float *rotZCur );
  void      findPosition( double relTime, float *posXCur, float *posYCur, float *posZCur );
  PointType deskewPoint( PointType *point, double relTime );
  void      projectPointCloud();
  void      cloudExtraction();
  void      publishClouds();
};

}  // namespace lio_sam

#endif  // IMAGE_PROJECTION_HPP