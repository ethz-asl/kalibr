cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(bsplines_python)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(bsplines REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(python_module REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)



include_directories()

# Set up the python exports.
SET(PY_PROJECT_NAME bsplines_python)
SET(PY_PACKAGE_DIR python/bsplines)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/SplinePython.cpp
  src/BSplinePython.cpp
  src/BSplinePosePython.cpp
)
ament_target_dependencies(${PY_PROJECT_NAME} bsplines numpy_eigen python_module sparse_block_matrix sm_common sm_kinematics)
ament_python_install_package(bsplines PACKAGE_DIR python/bsplines)

ament_package()
