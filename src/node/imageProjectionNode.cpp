#include "imageProjection.hpp"

int main( int argc, char **argv )
{
  ros::init( argc, argv, "lio_sam" );

  lio_sam::ImageProjection IP;

  ROS_INFO_STREAM( BOLDGREEN << "----> Image Projection Started." << RESET );

  ros::MultiThreadedSpinner spinner( 3 );
  spinner.spin();

  return 0;
}
