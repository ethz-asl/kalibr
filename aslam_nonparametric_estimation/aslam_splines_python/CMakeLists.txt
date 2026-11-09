cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_splines_python)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(aslam_splines REQUIRED)
find_package(bsplines_python REQUIRED)
find_package(bsplines REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(python_module REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(sm_timing REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_logging REQUIRED)
find_package(opencv2_catkin REQUIRED)



include_directories()

add_python_export_library(${PROJECT_NAME} python/aslam_splines
  src/spline_module.cpp
  src/BSplineMotionError.cpp
  src/SimpleSplineError.cpp
)
ament_target_dependencies(${PROJECT_NAME} aslam_splines bsplines_python bsplines numpy_eigen python_module sparse_block_matrix sm_common sm_kinematics aslam_backend_expressions aslam_backend aslam_backend_python sm_timing aslam_cameras aslam_time sm_opencv sm_property_tree sm_logging opencv2_catkin)
target_link_libraries(${PROJECT_NAME})

##################
## Installation ##
##################

# handled by add_python_export_library
ament_python_install_package(aslam_splines PACKAGE_DIR python/aslam_splines)

ament_package()
