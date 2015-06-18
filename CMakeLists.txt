cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(GTest REQUIRED)

catkin_package(
  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)


add_executable(perception_Run src/run.cc)
target_link_libraries(perception_Run ${GTEST_LIBRARIES} ${catkin_LIBRARIES} pthread)
add_dependencies(perception_Run perception_run_cc)

add_executable(Delaunay src/delaunay_shull.cc)
target_link_libraries(Delaunay ${GTEST_LIBRARIES} ${catkin_LIBRARIES} pthread)
add_dependencies(Delaunay perception_delaunay_cc)

catkin_add_gtest(perception_utest 
	src/nd_array.cc
	src/coordinate_list.cc
	src/mesh.cc
	src/mesh_triple.h
	src/triangle.cc
	src/triple.cc
	src/test/utest.cc
	)
target_link_libraries(perception_utest ${catkin_LIBRARIES})        
