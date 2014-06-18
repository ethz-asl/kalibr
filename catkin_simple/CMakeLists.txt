cmake_minimum_required(VERSION 2.8.3)
project(catkin_simple)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS catkin
  CFG_EXTRAS catkin_simple-extras.cmake
)
