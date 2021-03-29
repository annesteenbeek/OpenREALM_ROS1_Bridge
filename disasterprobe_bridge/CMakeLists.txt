cmake_minimum_required(VERSION 2.8.3)
project(disasterprobe_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
find_package(PCL 1.4 REQUIRED)
find_package(OCTOMAP REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  pcl_ros
  octomap_ros
  realm_msgs
  geometry_msgs
  std_msgs
  message_generation
  # realm_ros
)

find_package(cmake_modules REQUIRED)
# find_package(OpenREALM REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlanObservations.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  realm_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        # INCLUDE_DIRS
            # include
        # LIBRARIES
            # realm_ros_lib
            # realm_ros
            # ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            roslib
            std_msgs
            realm_msgs
            cmake_modules
            octomap_ros
            geometry_msgs
            message_runtime
            # realm_ros
            # pcl
            # pcl_ros
        DEPENDS
            # Boost
            # octomap
            # PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  # ${realm_ros_INCLUDE_DIRS}
  # ${OpenREALM_INCLUDE_DIRS}
)

# add_library(
#  ${realm_ros_LIBRARIES}
# )

add_executable(plan_observations src/plan_observations.cpp)
add_dependencies(plan_observations ${catkin_EXPORTED_TARGETS})
target_link_libraries (plan_observations ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})


