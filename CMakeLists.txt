cmake_minimum_required(VERSION 3.5)
project(aprilslamcpp)

# Set GTSAM_DIR to the directory containing GTSAMConfig.cmake
set(GTSAM_DIR "/home/shu/GTSAM/build")
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")

# Enable compile optimizations
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
# set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf2_ros
  roslib
  pcl_ros
  pcl_conversions
  apriltag_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED) 

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf2_ros nav_msgs apriltag_ros
  DEPENDS GTSAM
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# aprilslamcpp calibration executable
add_executable(aprilslamcpp_cal src/aprilslamcppcal.cpp src/publishing_utils.cpp)
target_link_libraries(
  aprilslamcpp_cal
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam 
  tbb
)

add_executable(aprilslamcpp_loc src/aprilslamcpploc.cpp src/publishing_utils.cpp)
target_link_libraries(
  aprilslamcpp_loc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam 
  tbb
)

#############
## Install ##
#############

install(
  TARGETS aprilslamcpp_cal aprilslamcpp_loc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_AprilSlamCPP.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
