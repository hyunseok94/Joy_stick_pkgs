cmake_minimum_required(VERSION 2.8.3)
project(joy_stick_pkgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(DEPENDS gazebo_ros roscpp rospy std_msgs)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(joy_talker src/joy_talker.cpp)
target_link_libraries(joy_talker ${catkin_LIBRARIES})
add_dependencies(joy_talker joy_stick_pkgs_generate_messages_cpp)
