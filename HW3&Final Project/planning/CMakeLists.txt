cmake_minimum_required(VERSION 3.0.2)
SET(CMAKE_BUILD_TYPE "Debug")
project(planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  waypoint_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS

  # INCLUDE_DIRS include
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIR}
)

add_executable(planning
  src/reference_line/reference_line.cpp
  src/collision_detection/collision_detection.cpp
  # src/Optimal_Frenet_Planner/Optimal_Frenet_Planner.cpp
  src/planning_node.cpp
  src/planning_main.cpp
  src/common.cpp
  src/Planner/EasyPlanner.cpp)

add_dependencies(planning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(cruise_speed_pub
  src/cruise_speed_pub.cpp)

add_dependencies(cruise_speed_pub
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(planning
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  OsqpEigen::OsqpEigen
)

target_link_libraries(cruise_speed_pub
  ${catkin_LIBRARIES}
)
