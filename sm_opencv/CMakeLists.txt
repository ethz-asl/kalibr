cmake_minimum_required(VERSION 2.8.3)
project(sm_opencv)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(${Boost_INCLUDE_DIRS})

cs_install()
cs_export()
