#include "mapOptmization.hpp"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "lio_sam" );

  lio_sam::MapOptimization MO;

  ROS_INFO_STREAM( BOLDGREEN << "----> Map Optimization Started." << RESET );

  std::thread loopthread( &lio_sam::MapOptimization::loopClosureThread, &MO );
  std::thread visualizeMapThread( &lio_sam::MapOptimization::visualizeGlobalMapThread, &MO );

  ros::spin();

  loopthread.join();
  visualizeMapThread.join();

  return 0;
}
