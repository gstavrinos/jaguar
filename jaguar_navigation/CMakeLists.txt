cmake_minimum_required(VERSION 2.8.3)
project(jaguar_navigation)

find_package(catkin REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

catkin_package()

