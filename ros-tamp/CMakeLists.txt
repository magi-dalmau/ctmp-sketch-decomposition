cmake_minimum_required(VERSION 2.8.3)
project(ros_tamp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

find_package(TinyXML REQUIRED)

find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
  INCLUDE_DIRS
    include
  DEPENDS
    TinyXML
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(${CMAKE_PROJECT_NAME}_test_node src/main.cpp) 
add_dependencies(${CMAKE_PROJECT_NAME}_test_node ${catkin_EXPORTED_TARGETS} ${TinyXML_LIBRARIES})
target_link_libraries(${CMAKE_PROJECT_NAME}_test_node ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})