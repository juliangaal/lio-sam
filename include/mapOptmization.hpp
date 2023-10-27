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

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "dataType.hpp"
#include "ivox3d/ivox3d.h"
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"
#include "statistics_accumulator.h"
#include "timer.h"
#include "utility.h"

// using namespace gtsam;

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::G;  // GPS pose
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
// ivox
using IVoxType    = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;


namespace lio_sam
{
class MapOptimization : public ParamServer
{
private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Timer
  AccumulateAverage timeAverage;
  lin::Timer        timerLin;

  // ivox
  bool                     needCorrectFlag = false;
  std::vector<PointVector> nearestCornerPoints;
  std::vector<PointVector> nearestSurfPoints;

  IVoxType::Options         iVoxOptions;
  std::shared_ptr<IVoxType> iVoxCornerMap = nullptr;
  std::shared_ptr<IVoxType> iVoxSurfMap   = nullptr;
  std::shared_ptr<IVoxType> iVoxGlobalMap = nullptr;

  // gtsam
  gtsam::NonlinearFactorGraph gtSAMgraph;
  gtsam::Values               initialEstimate;
  gtsam::Values               optimizedEstimate;
  gtsam::ISAM2*               isam;
  gtsam::Values               isamCurrentEstimate;
  Eigen::MatrixXd             poseCovariance;

  // ros
  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubLaserOdometryGlobal;
  ros::Publisher pubLaserOdometryIncremental;
  ros::Publisher pubKeyPoses;
  ros::Publisher pubKeyframePath;
  ros::Publisher pubRealtimePath;

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrames;
  ros::Publisher pubRecentKeyFrame;
  ros::Publisher pubCloudRegisteredRaw;
  ros::Publisher pubLoopConstraintEdge;

  ros::Publisher pubSLAMInfo;

  ros::Subscriber subCloud;
  ros::Subscriber subGPS;
  ros::Subscriber subLoop;

  ros::ServiceServer srvSaveMap;

  tf2_ros::TransformBroadcaster tfBroadcaster;

  // GPS
  bool                           firstGps;
  GeographicLib::LocalCartesian  gpsLocalCartesian;
  std::deque<nav_msgs::Odometry> gpsQueue;
  lio_sam::cloud_info            cloudInfo;

  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

  pcl::PointCloud<PointType>::Ptr     cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointType>::Ptr     copy_cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;    // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;      // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;  // downsampled corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;    // downsampled surf feature set from odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  std::vector<PointType> laserCloudOriCornerVec;  // corner point holder for parallel computation
  std::vector<PointType> coeffSelCornerVec;
  std::vector<bool>      laserCloudOriCornerFlag;
  std::vector<PointType> laserCloudOriSurfVec;  // surf point holder for parallel computation
  std::vector<PointType> coeffSelSurfVec;
  std::vector<bool>      laserCloudOriSurfFlag;

  std::map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
  pcl::PointCloud<PointType>::Ptr                                                  laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr                                                  laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr                                                  laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr                                                  laserCloudSurfFromMapDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterICP;
  pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of scan-to-map optimization

  bool               firstScanFlag = true;
  ros::Time          timeLaserInfoStamp;
  double             timeLaserInfoCur;
  float              searchRadiusTemp;
  nav_msgs::Odometry lastLaserOdometryROS;

  float transformTobeMapped[ 6 ];

  std::mutex mtx;
  std::mutex mtxLoopInfo;

  bool    isDegenerate = false;
  cv::Mat matP;

  int laserCloudCornerFromMapDSNum = 0;
  int laserCloudSurfFromMapDSNum   = 0;
  int laserCloudCornerLastDSNum    = 0;
  int laserCloudSurfLastDSNum      = 0;

  // loop closure
  bool                                                 aLoopIsClosed = false;
  std::map<int, int>                                   loopIndexContainer;  // from new to old
  std::vector<std::pair<int, int>>                     loopIndexQueue;
  std::vector<gtsam::Pose3>                            loopPoseQueue;
  std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
  std::deque<std_msgs::Float64MultiArray>              loopInfoVec;

  nav_msgs::Path keyFramePath;
  nav_msgs::Path realtimePath;

  Eigen::Affine3f transPointAssociateToMap;
  Eigen::Affine3f incrementalOdometryAffineFront;
  Eigen::Affine3f incrementalOdometryAffineBack;

  // timer
  faster_lio::Timer timer;

public:
  MapOptimization();
  ~MapOptimization();
  void                            allocateMemory();
  void                            laserCloudInfoHandler( const lio_sam::cloud_infoConstPtr& msgIn );
  void                            gpsHandler( const nav_msgs::Odometry::ConstPtr& gpsMsg );
  void                            gpsHandler( const sensor_msgs::NavSatFixConstPtr& gpsMsg );
  void                            pointAssociateToMap( PointType const* const pi, PointType* const po );
  pcl::PointCloud<PointType>::Ptr transformPointCloud( pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn );
  gtsam::Pose3                    pclPointTogtsamPose3( PointTypePose thisPoint );
  gtsam::Pose3                    trans2gtsamPose( float transformIn[] );
  Eigen::Affine3f                 pclPointToAffine3f( PointTypePose thisPoint );
  Eigen::Affine3f                 trans2Affine3f( float transformIn[] );
  PointTypePose                   trans2PointTypePose( float transformIn[] );
  bool                            saveMapService( lio_sam::save_mapRequest& req, lio_sam::save_mapResponse& res );
  void                            visualizeGlobalMapThread();
  void                            publishGlobalMap();
  void                            loopClosureThread();
  void                            loopInfoHandler( const std_msgs::Float64MultiArray::ConstPtr& loopMsg );
  void                            performLoopClosure();
  bool                            detectLoopClosureDistance( int* latestID, int* closestID );
  bool                            detectLoopClosureExternal( int* latestID, int* closestID );
  void                            loopFindNearKeyframes( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum );
  void                            visualizeLoopClosure();
  void                            updateInitialGuess();
  void                            extractForLoopClosure();
  void                            extractNearby();
  void                            extractCloud( pcl::PointCloud<PointType>::Ptr cloudToExtract );
  void                            extractCloudForIVox( pcl::PointCloud<PointType>::Ptr cloudToExtract );
  void                            candidatePointsForIVox( const PointVector& points );
  void                            extractSurroundingKeyFrames();
  void                            downsampleCurrentScan();
  void                            updatePointAssociateToMap();
  void                            cornerOptimization();
  void                            cornerOptimizationIVox();
  void                            surfOptimization();
  void                            surfOptimizationIVox();
  void                            combineOptimizationCoeffs();
  bool                            LMOptimization( int iterCount );
  void                            scan2MapOptimization();
  void                            scan2MapOptimizationIVox();
  void                            transformUpdate();
  bool                            saveFrame();
  void                            addOdomFactor();
  void                            addGPSFactor();
  void                            addLoopFactor();
  void                            saveKeyFramesAndFactor();
  void                            correctPoses();
  void                            updatePath( const PointTypePose& pose_in );
  void                            publishOdometry();
  void                            publishFrames();
  void                            publishTransform();
};
}  // namespace lio_sam
