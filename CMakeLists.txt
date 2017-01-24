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

add_definitions(-std=c++11)

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(state_publisher src/state_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})

add_executable(Evolve_node src/Evol.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Evolve_node ${catkin_LIBRARIES})

add_executable(Evolve_node_paral src/Evol_paral.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Evolve_node_paral ${catkin_LIBRARIES})

add_executable(Evolve_node_single src/Evol_single.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Evolve_node_single ${catkin_LIBRARIES})

add_executable(Evolve_node_2leg src/Evol_2leg.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Evolve_node_2leg ${catkin_LIBRARIES})

add_executable(Evolve_node_2leg_collision src/Evol_2leg_dyncollision.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Evolve_node_2leg_collision ${catkin_LIBRARIES})

add_executable(Evolve_node_4leg src/Evol_4leg.cpp src/Algorithm_lib/NEAT.cpp)
target_link_libraries(Evolve_node_4leg ${catkin_LIBRARIES})
