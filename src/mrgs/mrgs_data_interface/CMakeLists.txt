cmake_minimum_required(VERSION 2.4.6)
project(mrgs_data_interface)

find_package(catkin REQUIRED COMPONENTS roscpp wifi_comm std_msgs nav_msgs geometry_msgs message_generation message_runtime)

add_message_files(
    FILES
    ForeignMap.msg
    ForeignMapVector.msg
    LatestRobotPose.msg
    LocalMap.msg
    MacArray.msg
    NetworkMap.msg)

generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_library(lz4 src/lz4/lz4.c)
add_executable(data_interface_node src/data_interface_node.cpp)
target_link_libraries(data_interface_node lz4 ${catkin_LIBRARIES})
