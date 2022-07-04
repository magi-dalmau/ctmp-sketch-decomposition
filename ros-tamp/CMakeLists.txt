cmake_minimum_required(VERSION 2.8.3)
project(ros_tamp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_planning
  geometry_msgs
  visualization_msgs
  eigen_conversions
  tf_conversions
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
  geometric_shapes
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
)

find_package(TinyXML REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    eigen_conversions
    interactive_markers
    tf2_geometry_msgs
    sensor_msgs
  INCLUDE_DIRS
    include
  DEPENDS
    TinyXML
    EIGEN3
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(${CMAKE_PROJECT_NAME}_test_node src/main.cpp) 
add_dependencies(${CMAKE_PROJECT_NAME}_test_node ${catkin_EXPORTED_TARGETS} ${TinyXML_LIBRARIES})
target_link_libraries(${CMAKE_PROJECT_NAME}_test_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

add_executable(${CMAKE_PROJECT_NAME}_test_brfs src/test_brfs.cpp) 
add_dependencies(${CMAKE_PROJECT_NAME}_test_brfs ${catkin_EXPORTED_TARGETS})
target_link_libraries(${CMAKE_PROJECT_NAME}_test_brfs ${catkin_LIBRARIES})

add_executable(${CMAKE_PROJECT_NAME}_test_tiago_problem src/test_tiago_problem.cpp src/moveit_tamp_problem.cpp src/cluttered_tamp_problem.cpp src/non_monotonic_tamp_problem.cpp) 
add_dependencies(${CMAKE_PROJECT_NAME}_test_tiago_problem ${catkin_EXPORTED_TARGETS})
target_link_libraries(${CMAKE_PROJECT_NAME}_test_tiago_problem ${catkin_LIBRARIES})
