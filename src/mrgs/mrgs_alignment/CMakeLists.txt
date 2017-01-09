cmake_minimum_required(VERSION 2.4.6)
project(mrgs_alignment)

find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs nav_msgs message_generation message_runtime)
find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(
    FILES
    align.srv
)

generate_messages(
    DEPENDENCIES
    nav_msgs
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS  roscpp tf std_msgs nav_msgs message_generation message_runtime
    LIBRARIES       mapmerge
    INCLUDE_DIRS    include
    DEPENDS         OpenCV)

# Targets
add_library(mapmerge src/mapmerge/common.cpp src/mapmerge/grid_map.cpp src/mapmerge/hough.cpp src/mapmerge/io.cpp src/mapmerge/manipulatemap.cpp)
target_link_libraries(mapmerge ${OpenCV_LIBRARIES})

add_executable(align_node src/align_node.cpp)
target_link_libraries(align_node mapmerge   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(align_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
