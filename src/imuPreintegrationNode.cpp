#include "imuPreintegration.hpp"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "roboat_loam" );

  lio_sam::IMUPreintegration ImuP;

  ROS_INFO_STREAM( BOLDGREEN << "----> IMU Preintegration Started." << RESET );

  ros::MultiThreadedSpinner spinner( 4 );
  spinner.spin();

  return 0;
}
