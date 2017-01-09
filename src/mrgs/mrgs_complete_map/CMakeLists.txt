cmake_minimum_required(VERSION 2.4.6)
project(mrgs_complete_map)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs mrgs_alignment mrgs_data_interface message_generation message_runtime)

add_message_files(
    FILES
    LatestMapTF.msg
    )

include_directories(${catkin_INCLUDE_DIRS})

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

add_executable(complete_map_node        src/complete_map_node.cpp)
add_executable(complete_map_node_alt    src/complete_map_node_alt.cpp)
target_link_libraries(complete_map_node     ${catkin_LIBRARIES})
target_link_libraries(complete_map_node_alt ${catkin_LIBRARIES})
