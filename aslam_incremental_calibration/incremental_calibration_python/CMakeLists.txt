cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration_python)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(APPLE)

else()
  add_definitions(-std=c++0x)
endif()

add_python_export_library(${PROJECT_NAME} src/incremental_calibration
  src/module.cpp
  src/OptimizationProblem.cpp
  src/IncrementalEstimator.cpp
  src/LinearSolver.cpp
)

cs_install()
cs_export()
