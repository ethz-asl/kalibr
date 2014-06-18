cmake_minimum_required(VERSION 2.8.3)
project(bar)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(${Boost_INCLUDE_DIRS})

catkin_simple()

cs_add_library(bar src/hello.cpp)
target_link_libraries(bar ${Boost_LIBRARIES})

cs_install()

cs_export()
