cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_kinematics)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_random REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sm_common sm_eigen sm_boost)

include_directories(include ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
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
ament_target_dependencies(${PROJECT_NAME} sm_common sm_eigen sm_boost sm_random)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
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

endif()
ament_package()
