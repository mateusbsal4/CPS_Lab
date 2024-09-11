cmake_minimum_required(VERSION 2.8.3)
project(relampago)

find_package(catkin REQUIRED COMPONENTS   
  ackermann_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs)

generate_messages(
  DEPENDENCIES
  ackermann_msgs   nav_msgs   sensor_msgs   std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# IMPORTANT: you must copy and adapt the following lines for each node you want to add!
add_executable(gamepad src/gamepad.cpp)
add_executable(keyboard src/keyboard.cpp)
add_executable(pid src/pid.cpp)
add_executable(mux src/mux.cpp)
target_link_libraries(gamepad ${catkin_LIBRARIES})
target_link_libraries(keyboard ${catkin_LIBRARIES})
target_link_libraries(pid ${catkin_LIBRARIES})
target_link_libraries(mux ${catkin_LIBRARIES})
add_dependencies(gamepad relampago_generate_messages_cpp)
add_dependencies(keyboard relampago_generate_messages_cpp)
add_dependencies(pid relampago_generate_messages_cpp)
add_dependencies(mux relampago_generate_messages_cpp)
