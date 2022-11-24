cmake_minimum_required(VERSION 3.0.2)
project(aslam_cv_backend_python)

find_package(catkin REQUIRED COMPONENTS
  sm_common
  sm_python
  sm_opencv
  numpy_eigen
  python_module
  sparse_block_matrix
  aslam_backend
  aslam_backend_expressions
  aslam_backend_python
  opencv2_catkin
  aslam_cameras
  aslam_cv_error_terms
  aslam_splines
  aslam_cv_python
  aslam_cv_backend
  aslam_time
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_python_export_library(${PROJECT_NAME} python/aslam_cv_backend
  src/module.cpp  
  src/GridCalibration.cpp
  # src/CameraGeometryDesignVariableContainer.cpp
#  src/NCameraSystemDesignVariableContainer.cpp
)

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
