cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_backend)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_random REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sparse_block_matrix
    sm_boost
    sm_random
    sm_timing
    sm_logging
    sm_property_tree)

include_directories(include ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/MEstimatorPolicies.cpp
  src/JacobianContainer.cpp
  src/DesignVariable.cpp
  src/ErrorTerm.cpp
  src/OptimizationProblemBase.cpp
  src/Optimizer.cpp
  src/sparse_matrix_functions.cpp
  src/OptimizationProblem.cpp
  src/SimpleOptimizationProblem.cpp
  src/JacobianBuilder.cpp
  src/LinearSystemSolver.cpp
  src/BlockCholeskyLinearSystemSolver.cpp
  src/SparseCholeskyLinearSystemSolver.cpp
  src/SparseQrLinearSystemSolver.cpp
  src/Matrix.cpp
  src/DenseMatrix.cpp
  src/SparseBlockMatrixWrapper.cpp
  src/DenseQrLinearSystemSolver.cpp
  src/BlockCholeskyLinearSolverOptions.cpp
  src/SparseCholeskyLinearSolverOptions.cpp
  src/SparseQRLinearSolverOptions.cpp
  src/DenseQRLinearSolverOptions.cpp
  src/TrustRegionPolicy.cpp
  src/ErrorTermDs.cpp
  src/GaussNewtonTrustRegionPolicy.cpp
  src/LevenbergMarquardtTrustRegionPolicy.cpp
  src/Marginalizer.cpp
  src/MarginalizationPriorErrorTerm.cpp
  src/DogLegTrustRegionPolicy.cpp
  src/Optimizer2.cpp
)
ament_target_dependencies(${PROJECT_NAME} sparse_block_matrix sm_boost sm_random sm_timing sm_logging sm_property_tree)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES}
  ${CHOLMOD_LIBRARY}
  ${SUITESPARSE_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/JacobianContainer.cpp
    test/test_sparse_matrix_functions.cpp
    test/TestOptimizer.cpp
    test/TestOptimizationProblem.cpp
    test/CompressedColumnMatrixTest.cpp
    test/MatrixTestHarness.cpp
    test/DenseMatrixTest.cpp
    test/SparseMatrixTest.cpp
    test/LinearSolverTests.cpp
    test/ErrorTermTests.cpp
    )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
ament_package()
