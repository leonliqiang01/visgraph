cmake_minimum_required(VERSION 2.8.3)
project(visgraph)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} geos-config --cflags")
set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} geos-config --ldflags")

find_package(catkin REQUIRED
	roscpp
	rospy
	visualization_msgs
)
find_package(Eigen3 3.1.0 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visgraph
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories( ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  include
)

link_directories(
  ${EIGEN3_LIBRARY_DIRS}
)
add_executable(visgraph src/main.cpp src/visgraph.cpp src/astar.cpp)
add_dependencies(visgraph visgraph_generate_messages_cpp)
target_link_libraries(visgraph ${catkin_LIBRARIES} geos)