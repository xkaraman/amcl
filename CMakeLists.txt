cmake_minimum_required(VERSION 2.8.3)
project(amcl_lib_pf)

# Build Particle Filter Library REFERENCED AS PF
add_subdirectory(include/libPF)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  octomap_ros
  octomap_msgs
  pcl_conversions
  pcl_ros
  cmake_modules
  tf2_geometry_msgs
  tf2
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES} )

find_package(OCTOMAP REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amcl_lib_pf
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 src 
 include/libPF/include
 include/libColor/src
)

set(amcl_lib_pf_SOURCES 
	src/amcl_lib_pf_node.cpp
	src/RobotState.cpp
    src/AMCLDepth.cpp
    src/MapModel.cpp
    src/RobotMovementModel.cpp
    src/RobotObservationModel.cpp
    src/RobotStateDistribution.cpp
    )


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/amcl_lib_pf.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node ${amcl_lib_pf_SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node  
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   PF
 )


