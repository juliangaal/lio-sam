#include "transformFusion.hpp"

namespace lio_sam
{
TransformFusion::TransformFusion()
{
  if ( lidarFrame != baselinkFrame )
  {
    try
    {
      tfListener.waitForTransform( lidarFrame, baselinkFrame, ros::Time( 0 ), ros::Duration( 3.0 ) );
      tfListener.lookupTransform( lidarFrame, baselinkFrame, ros::Time( 0 ), lidar2Baselink );
    }
    catch ( tf::TransformException& ex )
    {
      ROS_ERROR( "%s", ex.what() );
    }
  }

  subLaserOdometry = nh.subscribe<nav_msgs::Odometry>( "lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay() );
  subImuOdometry   = nh.subscribe<nav_msgs::Odometry>( odomTopic + "_incremental", 2000, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay() );

  pubImuOdometry = nh.advertise<nav_msgs::Odometry>( odomTopic, 2000 );
  pubImuPath     = nh.advertise<nav_msgs::Path>( "lio_sam/imu/path", 1 );
}

void TransformFusion::lidarOdometryHandler( const nav_msgs::Odometry::ConstPtr& odomMsg )
{
  std::lock_guard<std::mutex> lock( mtx );

  lidarOdomAffine = odom2affine( *odomMsg );

  lidarOdomTime = odomMsg->header.stamp.toSec();
}

void TransformFusion::imuOdometryHandler( const nav_msgs::Odometry::ConstPtr& odomMsg )
{
  // static tf
  static tf::TransformBroadcaster tfMap2Odom;
  static tf::Transform            map_to_odom = tf::Transform( tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Vector3( 0, 0, 0 ) );
  tfMap2Odom.sendTransform( tf::StampedTransform( map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame ) );

  std::lock_guard<std::mutex> lock( mtx );

  imuOdomQueue.push_back( *odomMsg );

  // get latest odometry (at current IMU stamp)
  if ( lidarOdomTime == -1 )
  {
    return;
  }
  while ( !imuOdomQueue.empty() )
  {
    if ( imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime )
    {
      imuOdomQueue.pop_front();
    }
    else
    {
      break;
    }
  }
  Eigen::Affine3f imuOdomAffineFront = odom2affine( imuOdomQueue.front() );
  Eigen::Affine3f imuOdomAffineBack  = odom2affine( imuOdomQueue.back() );
  Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
  Eigen::Affine3f imuOdomAffineLast  = lidarOdomAffine * imuOdomAffineIncre;
  float           x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles( imuOdomAffineLast, x, y, z, roll, pitch, yaw );

  // publish latest odometry
  nav_msgs::Odometry laserOdometry    = imuOdomQueue.back();
  laserOdometry.pose.pose.position.x  = x;
  laserOdometry.pose.pose.position.y  = y;
  laserOdometry.pose.pose.position.z  = z;
  laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( roll, pitch, yaw );
  pubImuOdometry.publish( laserOdometry );

  // publish tf
  static tf::TransformBroadcaster tfOdom2BaseLink;
  tf::Transform                   tCur;
  tf::poseMsgToTF( laserOdometry.pose.pose, tCur );
  if ( lidarFrame != baselinkFrame )
  {
    tCur = tCur * lidar2Baselink;
  }
  tf::StampedTransform odom_2_baselink = tf::StampedTransform( tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame );
  tfOdom2BaseLink.sendTransform( odom_2_baselink );

  // publish IMU path
  static nav_msgs::Path imuPath;
  static double         last_path_time = -1;
  double                imuTime        = imuOdomQueue.back().header.stamp.toSec();
  if ( imuTime - last_path_time > 0.1 )
  {
    last_path_time = imuTime;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp    = imuOdomQueue.back().header.stamp;
    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose            = laserOdometry.pose.pose;
    imuPath.poses.push_back( pose_stamped );
    while ( !imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0 )
    {
      imuPath.poses.erase( imuPath.poses.begin() );
    }
    if ( pubImuPath.getNumSubscribers() != 0 )
    {
      imuPath.header.stamp    = imuOdomQueue.back().header.stamp;
      imuPath.header.frame_id = odometryFrame;
      pubImuPath.publish( imuPath );
    }
  }
}
}  // namespace lio_sam