cmake_minimum_required(VERSION 2.4.6)
project(wifi_comm)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
    FILES
    WiFiCommContainer.msg
    WiFiNeighbour.msg
    WiFiNeighboursList.msg
)

generate_messages(DEPENDENCIES  std_msgs)
catkin_package(
    CATKIN_DEPENDS  roscpp std_msgs message_generation message_runtime
    INCLUDE_DIRS    include
    LIBRARIES   wifi_comm)

# Targets
add_library(wifi_comm src/wifi_comm_lib.cpp)
target_link_libraries(wifi_comm ${catkin_LIBRARIES})
add_dependencies(wifi_comm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wifi_discovery_node src/wifi_discovery_node.cpp)
target_link_libraries(wifi_discovery_node ${catkin_LIBRARIES})
add_dependencies(wifi_discovery_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
