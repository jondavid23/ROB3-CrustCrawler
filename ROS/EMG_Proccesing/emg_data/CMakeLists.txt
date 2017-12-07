################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(emg_data)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamixel_workbench_controllers
 CATKIN_DEPENDS roscpp std_msgs 
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(emg_data src/emg_data.cpp)
target_link_libraries(emg_data ${catkin_LIBRARIES})


################################################################################
# Install
################################################################################

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
