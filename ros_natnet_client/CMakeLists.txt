cmake_minimum_required(VERSION 2.8.0)
project(ros_natnet_client)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Pangolin REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(FMT REQUIRED)
find_package(Eigen REQUIRED)
include_directories(
${Eigen_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
)

cs_add_library(Mocap
  src/Mocap.cc
)

cs_add_executable(ros_natnet_client
	src/ros_natnet_client.cpp
)


#target_link_libraries( Mocap ${catkin_LIBRARIES} ${Pangolin_LIBRARIES}  fmt)
#target_link_libraries( ros_natnet_client  ${catkin_LIBRARIES} ${Pangolin_LIBRARIES}  fmt)
target_link_libraries(ros_natnet_client Mocap ${catkin_LIBRARIES} NatNet.so ${Pangolin_LIBRARIES}  fmt)


cs_install()
cs_export()

