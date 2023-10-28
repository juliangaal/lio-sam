#include "featureExtraction.hpp"

int main( int argc, char **argv )
{
  ros::init( argc, argv, "lio_sam" );

  lio_sam::FeatureExtraction FE;

  ROS_INFO_STREAM( BOLDGREEN << "----> Feature Extraction Started." << RESET );

  ros::spin();

  return 0;
}