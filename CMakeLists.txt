cmake_minimum_required(VERSION 2.8.3)
project(crab_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()


include_directories( ${catkin_INCLUDE_DIRS})

add_executable(state_publisher src/state_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})
