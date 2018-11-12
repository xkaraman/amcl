cmake_minimum_required(VERSION 2.8.3)
project(amcl_lib_pf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build Particle Filter Library
add_subdirectory(include/libPF)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  roscpp
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
link_libraries(${OCTOMAP_LIBRARIES})

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

SET(LIBRARIES MapModel MotionModel LaserObsModel ObservationModel RGBObsModel RobotState RobotStateDistribution)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${LIBRARIES} amclDepth
#  CATKIN_DEPENDS tf2
#  DEPENDS OCTOMAP PCL
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
 #include/libColor/src
)


## Declare a C++ library
add_library(MapModel	src/MapModel.cpp)
add_library(MotionModel	src/RobotMovementModel.cpp)
#target_link_libraries(MotionModel PF)

add_library(ObservationModel	src/RobotObservationModel.cpp)
#target_link_libraries(ObservationModel PF)

add_library(LaserObsModel src/LaserObservationModel.cpp)
#target_link_libraries(LaserObsModel PF)

add_library(RGBObsModel src/RGBObservationModel.cpp)
#target_link_libraries(RGBObsModel PF)

add_library(RobotState src/RobotState.cpp)
add_library(RobotStateDistribution src/RobotStateDistribution.cpp)
#target_link_libraries(RobotStateDistribution PF)

add_library(AMCLDepth src/AMCLDepth.cpp)
target_link_libraries(AMCLDepth PF ${LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#set(amcl_lib_pf_SOURCES 
#	src/amcl_lib_pf_node.cpp
#	src/RobotState.cpp
#    src/AMCLDepth.cpp
#    src/MapModel.cpp
#    src/RobotMovementModel.cpp
#    src/FeatureMatching.cpp
#    src/RobotObservationModel.cpp
#    src/RobotStateDistribution.cpp
#    )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/amcl_lib_pf_node.cpp)
target_link_libraries(${PROJECT_NAME}_node  
   ${catkin_LIBRARIES}
   AMCLDepth
)


