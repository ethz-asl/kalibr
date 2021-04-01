cmake_minimum_required(VERSION 3.0.2)
project(aslam_backend_expressions)

set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS aslam_backend sm_boost sm_random sm_kinematics sparse_block_matrix sm_timing)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sparse_block_matrix
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  # catkin_add_gtest(${PROJECT_NAME}_test
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
