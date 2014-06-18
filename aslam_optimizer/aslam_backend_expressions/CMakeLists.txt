cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend_expressions)

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin_simple REQUIRED)

catkin_simple()

include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

cs_add_library(${PROJECT_NAME}
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
  )

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/RotationExpression.cpp
  test/HomogeneousExpression.cpp
  test/MatrixAndEuclideanExpression.cpp
  # \todo reenable after I can talk to Hannes.
  test/FixedPointNumberTest.cpp
  test/GenericScalarExpressionTest.cpp
  test/GenericMatrixExpression.cpp
  test/QuaternionExpression.cpp
  test/ScalarExpression.cpp
  test/ErrorTest_Transformation.cpp
  test/ErrorTest_Euclidean.cpp
  )

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()

cs_export()

