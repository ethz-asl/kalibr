cmake_minimum_required(VERSION 3.0.2)
project(sm_python)

set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS sm_common numpy_eigen sm_kinematics 
sm_timing sm_logging sm_matrix_archive
sm_property_tree python_module numpy_eigen
)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS sm_common numpy_eigen sm_kinematics sm_timing sm_logging
  sm_matrix_archive sm_property_tree python_module
)
add_definitions(-D__STRICT_ANSI__)
  
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

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
target_link_libraries(${PY_PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
