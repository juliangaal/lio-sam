cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

set(GLOG_INCLUDE_DIRS /usr/local/include/glog)
set(GLOG_LIBRARIES /usr/local/lib/x86_64-linux-gnu/libglog.so)

find_package(OpenMP REQUIRED)
message("****OpenMP_INCLUDE_DIRS: ${OpenMP_INCLUDE_DIRS}")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

find_package(PCL REQUIRED QUIET)
message("****PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
find_package(OpenCV REQUIRED QUIET)
message("****OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
find_package(GTSAM REQUIRED QUIET)
message("****GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")
find_package(Boost REQUIRED COMPONENTS timer)
message("****Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
  ${catkin_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# timer
add_library(timer src/timer.cpp)
target_link_libraries(timer glog)

add_library(gtsamGravityFactor 
  src/gravityFactor.cpp
  src/gravityEstimator.cpp
  # src/gtsamGravityFactor.cpp
  )
target_link_libraries(gtsamGravityFactor gtsam glog)


# Range Image Projection
add_library(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(imageProjectionNode src/node/imageProjectionNode.cpp)
add_dependencies(imageProjectionNode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imageProjectionNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam imageProjection)

# Feature Extraction
add_library(featureExtraction src/featureExtraction.cpp)
target_link_libraries(featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
add_dependencies(featureExtraction  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(featureExtractionNode src/node/featureExtractionNode.cpp)
add_dependencies(featureExtractionNode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(featureExtractionNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam  featureExtraction)

# Mapping Optimization
add_library(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam timer)
add_dependencies(mapOptmization  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(mapOptmizationNode src/node/mapOptmizationNode.cpp)
add_dependencies(mapOptmizationNode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(mapOptmizationNode PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(mapOptmizationNode Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam mapOptmization)

# IMU Preintegration
add_library(imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(imuPreintegration Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam gtsamGravityFactor)
add_dependencies(imuPreintegration  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(imuPreintegrationNode src/node/imuPreintegrationNode.cpp)
add_dependencies(imuPreintegrationNode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imuPreintegrationNode Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} gtsam gtsamGravityFactor imuPreintegration )

# Transform Fusion
add_library(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
add_dependencies(transformFusion  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(transformFusionNode src/node/transformFusionNode.cpp)
add_dependencies(transformFusionNode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(transformFusionNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam transformFusion )

install(TARGETS imageProjectionNode featureExtractionNode mapOptmizationNode imuPreintegrationNode transformFusionNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)