cmake_minimum_required(VERSION 2.8.3)
project(intensity_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  message_generation
  cv_bridge
  image_transport
  eigen_conversions
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

add_message_files(
  FILES
  LoopInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_executable(intensity_slam_laser_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
target_link_libraries(intensity_slam_laser_processing_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(intensity_slam_odom_estimation_node src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
target_link_libraries(intensity_slam_odom_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(intensity_slam_isc_generation_node src/iscGenerationNode.cpp src/iscGenerationClass.cpp src/lidar.cpp)
target_link_libraries(intensity_slam_isc_generation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
add_dependencies(intensity_slam_isc_generation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(intensity_slam_isc_optimization_node src/iscOptimizationNode.cpp src/iscOptimizationClass.cpp src/lidarOptimization.cpp src/lidar.cpp)
target_link_libraries(intensity_slam_isc_optimization_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
add_dependencies(intensity_slam_isc_optimization_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
