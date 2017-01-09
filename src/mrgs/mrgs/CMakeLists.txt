cmake_minimum_required(VERSION 2.8.3)
project(mrgs)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES catbot_description
   CATKIN_DEPENDS mrgs_alignment mrgs_auxiliary_nodes mrgs_complete_map mrgs_scripts
#  DEPENDS system_lib
)
