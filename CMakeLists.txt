cmake_minimum_required(VERSION 2.8.3)
project(p4_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mg_msgs
  roscpp
  std_msgs
  cmake_modules
)

find_package(osqp REQUIRED)

find_package(Eigen REQUIRED)

add_subdirectory(P4)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS mg_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  P4/src
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(p4_services src/p4_services_node.cpp
                           src/services.cpp
                           src/p4_helper.cpp
                           P4/src/polynomial_solver.cc)
add_dependencies(p4_services ${catkin_EXPORTED_TARGETS})
target_link_libraries(p4_services ${catkin_LIBRARIES} osqp::osqp)