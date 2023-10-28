#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <pcl/search/impl/search.hpp>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "utility/color.h"


// float magicSqrt( const float &x )
// {
//   float temp  = x;
//   float xHalf = 0.5f * temp;
//   int   i     = *(int *)( &temp );
//   i           = 0x5f3759df - ( i >> 1 );
//   temp        = *(float *)( &i );
//   temp        = 0.5f * ( temp + xHalf / temp );
//   return temp;
// }

template <typename T>
sensor_msgs::PointCloud2 publishCloud( const ros::Publisher &thisPub, const T &thisCloud, ros::Time thisStamp, std::string thisFrame )
{
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg( *thisCloud, tempCloud );
  tempCloud.header.stamp    = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if ( thisPub.getNumSubscribers() != 0 )
  {
    thisPub.publish( tempCloud );
  }
  return tempCloud;
}

template <typename T>
double ROS_TIME( T msg )
{
  return msg->header.stamp.toSec();
}


template <typename T>
void imuAngular2rosAngular( sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z )
{
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}


template <typename T>
void imuAccel2rosAccel( sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z )
{
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}


template <typename T>
void imuRPY2rosRPY( sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw )
{
  double         imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF( thisImuMsg->orientation, orientation );
  tf::Matrix3x3( orientation ).getRPY( imuRoll, imuPitch, imuYaw );

  *rosRoll  = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw   = imuYaw;
}


inline float pointDistance( PointType p )
{
  return sqrt( p.x * p.x + p.y * p.y + p.z * p.z );
}


inline float pointDistance( PointType p1, PointType p2 )
{
  return sqrt( ( p1.x - p2.x ) * ( p1.x - p2.x ) + ( p1.y - p2.y ) * ( p1.y - p2.y ) + ( p1.z - p2.z ) * ( p1.z - p2.z ) );
}

inline float constraintTransformation( float value, float limit )
{
  if ( value < -limit )
  {
    value = -limit;
  }
  if ( value > limit )
  {
    value = limit;
  }
  return value;
}

Eigen::Affine3f odom2affine( nav_msgs::Odometry odom )
{
  double x, y, z, roll, pitch, yaw;
  x = odom.pose.pose.position.x;
  y = odom.pose.pose.position.y;
  z = odom.pose.pose.position.z;

  tf2::Quaternion q( odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w );
  tf2::Matrix3x3  mat( q );
  mat.getRPY( roll, pitch, yaw );

  return pcl::getTransformation( x, y, z, roll, pitch, yaw );
}

#endif
