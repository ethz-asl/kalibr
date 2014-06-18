cmake_minimum_required(VERSION 2.8.3)
project(sm_python)

#catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS sm_common numpy_eigen sm_kinematics 
                                        sm_timing sm_logging sm_matrix_archive
                                        sm_property_tree python_module)
include_directories(${catkin_INCLUDE_DIRS})
find_package(Eigen REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include  ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sm_common numpy_eigen sm_kinematics sm_timing sm_logging
                 sm_matrix_archive sm_property_tree python_module
  DEPENDS
)
add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(include)

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

target_link_libraries(${PY_PROJECT_NAME} ${Boost_LIBRARIES})
