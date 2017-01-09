cmake_minimum_required(VERSION 2.4.6)
project(foreign_relay)

find_package(catkin REQUIRED COMPONENTS roscpp topic_tools xmlrpcpp)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS  roscpp roscpp topic_tools xmlrpcpp
)

add_executable(foreign_relay src/foreign_relay.cpp)
target_link_libraries(foreign_relay ${catkin_LIBRARIES})
