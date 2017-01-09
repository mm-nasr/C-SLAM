cmake_minimum_required(VERSION 2.4.6)
project(mrgs_auxiliary_nodes)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs wifi_comm mrgs_complete_map mrgs_data_interface tf)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

add_executable(remote_nav_node src/remote_nav_node.cpp)
add_executable(map_dam_node src/map_dam_node.cpp)
target_link_libraries(remote_nav_node ${catkin_LIBRARIES})
target_link_libraries(map_dam_node ${catkin_LIBRARIES})
