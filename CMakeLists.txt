cmake_minimum_required(VERSION 2.8.3)
project(Perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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


add_executable(Test src/Test.cpp)
target_link_libraries(Test ${GTEST_LIBRARIES} ${catkin_LIBRARIES} pthread)
add_dependencies(Test Perception_test_cpp)

catkin_add_gtest(utest src/test/utest.cpp src/Foo.cpp)
target_link_libraries(utest ${catkin_LIBRARIES})        
