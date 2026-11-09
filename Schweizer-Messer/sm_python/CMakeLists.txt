cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_python)

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sm_common REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_matrix_archive REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(sm_common numpy_eigen sm_kinematics sm_timing sm_logging
  sm_matrix_archive sm_property_tree python_module)
add_definitions(-D__STRICT_ANSI__)
  
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Set up the python exports.
SET(PY_PROJECT_NAME sm_python)
SET(PY_PACKAGE_DIR python/sm)

find_package(Boost REQUIRED COMPONENTS system serialization)

##############
## Building ##
##############
add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cpp
  src/export_rotational_kinematics.cpp
  src/export_rotations.cpp
  src/export_transformations.cpp
  src/export_quaternion_algebra.cpp
  src/export_homogeneous.cpp
  src/exportTransformation.cpp
  src/exportHomogeneousPoint.cpp
  src/exportTimestampCorrector.cpp
  src/exportPropertyTree.cpp
  src/exportEigen.cpp
  src/exportUncertainVector.cpp
  src/exportMatrixArchive.cpp
  src/export_kinematics_property_tree.cpp
  src/export_eigen_property_tree.cpp
  src/Logging.cpp
  src/exportNsecTime.cpp
  src/random.cpp
)
ament_target_dependencies(${PY_PROJECT_NAME} sm_common numpy_eigen sm_kinematics sm_timing sm_logging sm_matrix_archive sm_property_tree python_module)
target_link_libraries(${PY_PROJECT_NAME} ${Boost_LIBRARIES})

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION include
)
ament_python_install_package(sm PACKAGE_DIR python/sm)

ament_package()
