cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_backend_expressions)

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_random REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_timing REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sparse_block_matrix)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/RotationExpression.cpp
  src/RotationExpressionNode.cpp
  src/RotationQuaternion.cpp
  src/MappedRotationQuaternion.cpp
  src/Vector2RotationQuaternionExpressionAdapter.cpp

  src/MatrixExpression.cpp
  src/MatrixExpressionNode.cpp
  src/MatrixTransformation.cpp

  src/TransformationExpression.cpp

  src/TransformationExpressionNode.cpp
  src/TransformationBasic.cpp

  src/EuclideanExpression.cpp
  src/EuclideanExpressionNode.cpp
  src/EuclideanPoint.cpp
  src/MappedEuclideanPoint.cpp

  src/HomogeneousExpression.cpp
  src/HomogeneousExpressionNode.cpp
  src/HomogeneousPoint.cpp
  src/MappedHomogeneousPoint.cpp

  src/ScalarExpression.cpp
  src/ScalarExpressionNode.cpp
  src/Scalar.cpp

  src/EuclideanDirection.cpp

  src/ErrorTermTransformation.cpp
  src/ErrorTermEuclidean.cpp

  src/MapTransformation.cpp

  src/MatrixBasic.cpp
  src/MatrixExpression.cpp
  src/MatrixExpressionNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} aslam_backend sm_boost sm_random sm_kinematics sparse_block_matrix sm_timing)
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

  # ament_add_gtest(${PROJECT_NAME}_test
  #   test/test_main.cpp
  #   test/RotationExpression.cpp
  #   test/HomogeneousExpression.cpp
  #   test/MatrixAndEuclideanExpression.cpp
  #   # \todo reenable after I can talk to Hannes.
  #   # test/FixedPointNumberTest.cpp
  #   test/GenericScalarExpressionTest.cpp
  #   test/GenericMatrixExpression.cpp
  #   test/QuaternionExpression.cpp
  #   test/ScalarExpression.cpp
  #   test/ErrorTest_Transformation.cpp
  #   test/ErrorTest_Euclidean.cpp
  #   )
  # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()
ament_package()
