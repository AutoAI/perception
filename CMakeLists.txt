cmake_minimum_required(VERSION 2.8.3)
project(Perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
)


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})
add_dependencies(Test Perception_generate_messages_cpp)
