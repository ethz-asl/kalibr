cmake_minimum_required(VERSION 2.8.3)
project(incremental_calibration)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../cmake/)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

cs_add_library(${PROJECT_NAME}
  src/base/Serializable.cpp
  src/base/Timestamp.cpp
  src/base/Condition.cpp
  src/base/Mutex.cpp
  src/base/Thread.cpp
  src/base/Threads.cpp
  src/base/Timer.cpp
  src/exceptions/Exception.cpp
  src/exceptions/InvalidOperationException.cpp
  src/exceptions/NullPointerException.cpp
  src/exceptions/SystemException.cpp
  src/statistics/NormalDistribution1v.cpp
  src/statistics/ChiSquareDistribution.cpp
  src/statistics/EstimatorMLNormal1v.cpp
  src/functions/IncompleteGammaPFunction.cpp
  src/functions/IncompleteGammaQFunction.cpp
  src/functions/LogFactorialFunction.cpp
  src/functions/LogGammaFunction.cpp
  src/core/IncrementalEstimator.cpp
  src/core/OptimizationProblem.cpp
  src/core/IncrementalOptimizationProblem.cpp
  src/core/LinearSolver.cpp
  src/core/LinearSolverOptions.cpp
  src/algorithms/marginalize.cpp
  src/algorithms/linalg.cpp
)

find_package(Boost REQUIRED COMPONENTS system thread)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

# Avoid clash with tr1::tuple:
# https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/VectorDesignVariableTest.cpp
  test/AlgorithmsTest.cpp
  test/OptimizationProblemTest.cpp
  test/IncrementalOptimizationProblemTest.cpp
  test/LinearSolverTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
