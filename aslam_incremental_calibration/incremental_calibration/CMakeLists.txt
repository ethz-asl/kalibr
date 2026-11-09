cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(incremental_calibration)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sm_eigen)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
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
ament_target_dependencies(${PROJECT_NAME} sm_eigen aslam_backend)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

# if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
#   # Avoid clash with tr1::tuple:
#   # https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#   add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#   ament_add_gtest(${PROJECT_NAME}_test
#     test/test_main.cpp
#     test/VectorDesignVariableTest.cpp
#     test/AlgorithmsTest.cpp
#     test/OptimizationProblemTest.cpp
#     test/IncrementalOptimizationProblemTest.cpp
#     test/LinearSolverTest.cpp
#     )
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
# endif()

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
ament_package()
