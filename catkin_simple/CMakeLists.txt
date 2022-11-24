cmake_minimum_required(VERSION 3.0.2)
project(catkin_simple)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS catkin
  CFG_EXTRAS catkin_simple-extras.cmake
)
