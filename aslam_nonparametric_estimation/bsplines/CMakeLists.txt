cmake_minimum_required(VERSION 2.8.3)
project(bsplines)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_simple()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME} 
  src/BSpline.cpp
  src/BSplinePose.cpp
)


cs_install()
cs_export()
