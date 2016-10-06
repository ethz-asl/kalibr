cmake_minimum_required(VERSION 2.8.3)
project(sm_kinematics)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

add_definitions(-Wall -std=c++0x)
include_directories(include ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

cs_add_library(${PROJECT_NAME}
  src/quaternion_algebra.cpp
  src/rotations.cpp
  src/transformations.cpp
  src/RotationalKinematics.cpp
  src/RotationVector.cpp
  src/EulerAnglesZYX.cpp
  src/EulerAnglesYawPitchRoll.cpp
  src/EulerRodriguez.cpp
  src/Transformation.cpp
  src/homogeneous_coordinates.cpp
  src/HomogeneousPoint.cpp
  src/UncertainTransformation.cpp
  src/UncertainHomogeneousPoint.cpp
  src/three_point_methods.cpp
  src/EulerAnglesZXY.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#############
## Testing ##
#############
# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/RotationalKinematicsTests.cpp
  test/QuaternionTests.cpp
  test/TransformationTests.cpp
  test/transformations.cpp
  test/HomogeneousPoint.cpp
  test/UncertainHomogeneousPoint.cpp
  test/test_main.cpp
  test/UncertainTransformationTests.cpp
  test/homogeneous_coordinates.cpp
  test/three_point_methods.cpp
 )
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
                        ${PROJECT_NAME})
endif()

cs_install()
cs_export()
