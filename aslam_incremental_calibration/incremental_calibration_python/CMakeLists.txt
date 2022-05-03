cmake_minimum_required(VERSION 3.0.2)
project(incremental_calibration_python)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS aslam_backend incremental_calibration sm_property_tree python_module numpy_eigen)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_python_export_library(${PROJECT_NAME} src/incremental_calibration
  src/module.cpp
  src/OptimizationProblem.cpp
  src/IncrementalEstimator.cpp
  src/LinearSolver.cpp
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
