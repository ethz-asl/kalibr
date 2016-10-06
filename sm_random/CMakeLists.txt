cmake_minimum_required(VERSION 2.8.3)
project(sm_random)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/random.cpp
)

cs_install()
cs_export()