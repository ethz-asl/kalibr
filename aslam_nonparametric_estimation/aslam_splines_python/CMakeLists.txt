cmake_minimum_required(VERSION 3.0.2)
project(aslam_splines_python)

find_package(catkin REQUIRED COMPONENTS
  aslam_splines
  bsplines_python
  bsplines
  numpy_eigen
  python_module
  sparse_block_matrix
  sm_common
  sm_kinematics
  aslam_backend_expressions
  aslam_backend
  aslam_backend_python
  sm_timing
  aslam_cameras
  aslam_time
  sm_opencv
  sm_property_tree
  sm_logging
  opencv2_catkin
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_python_export_library(${PROJECT_NAME} python/aslam_splines
  src/spline_module.cpp
  src/BSplineMotionError.cpp
  src/SimpleSplineError.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

##################
## Installation ##
##################

# handled by add_python_export_library
