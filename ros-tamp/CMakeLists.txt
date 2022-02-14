cmake_minimum_required(VERSION 2.8.3)
project(ros_tamp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(catkin REQUIRED COMPONENTS

	std_msgs	
	roslib
  roscpp  
  eigen_conversions
  tf_conversions
  kdl_parser  
)


catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
  roscpp 	
  roslib  
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)


##SKELETON FUSION NODE
add_executable(${CMAKE_PROJECT_NAME}_skeleton_fusion_node src/skeleton_fusion_node.cpp src/tracking_fusion.cpp src/human_modeling.cpp src/particle_filter.cpp src/uk_filter.cpp src/hk_filter.cpp src/munkres.cpp) 
target_link_libraries(${CMAKE_PROJECT_NAME}_skeleton_fusion_node ${catkin_LIBRARIES})
add_dependencies(${CMAKE_PROJECT_NAME}_skeleton_fusion_node ${catkin_EXPORTED_TARGETS})
