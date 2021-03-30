cmake_minimum_required(VERSION 3.0.2)
project(incremental_calibration_python)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_python_export_library(${PROJECT_NAME} src/incremental_calibration
  src/module.cpp
  src/OptimizationProblem.cpp
  src/IncrementalEstimator.cpp
  src/LinearSolver.cpp
)

cs_install()
cs_export()
