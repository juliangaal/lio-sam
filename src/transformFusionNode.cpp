#include "transformFusion.hpp"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "roboat_loam" );

  lio_sam::TransformFusion TF;

  ROS_INFO_STREAM( BOLDGREEN << "----> Transform Fusion Started." << RESET );

  ros::MultiThreadedSpinner spinner( 4 );
  spinner.spin();

  return 0;
}
