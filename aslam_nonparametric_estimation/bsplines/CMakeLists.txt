cmake_minimum_required(VERSION 3.0.2)
project(bsplines)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME} 
  src/BSpline.cpp
  src/BSplinePose.cpp
)


cs_install()
cs_export()
