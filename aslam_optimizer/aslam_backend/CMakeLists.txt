cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${Eigen_INCLUDE_DIRS})

add_definitions( -std=c++0x )

# enable timing via cmake option -Daslam_backend_ENABLE_TIMING

SET(aslam_backend_ENABLE_TIMING OFF CACHE BOOL "Enable sm::timing based profiling in aslam_backend")
# cmake -Daslam_backend_ENABLE_TIMING=1 or catkin build -Daslam_backend_ENABLE_TIMING
if (${PROJECT_NAME}_ENABLE_TIMING)
  message(STATUS "${PROJECT_NAME}: Timing enabled")
  add_definitions(-D${PROJECT_NAME}_ENABLE_TIMING)
endif()

cs_add_library(${PROJECT_NAME}
  src/MEstimatorPolicies.cpp
  src/JacobianContainerSparse.cpp
  src/JacobianContainerDense.cpp
  src/DesignVariable.cpp
  src/ErrorTerm.cpp
  src/ScalarNonSquaredErrorTerm.cpp
  src/OptimizationProblemBase.cpp
  src/LineSearch.cpp
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
  src/SamplerBase.cpp
  src/OptimizerBase.cpp
  src/Optimizer2.cpp
  src/OptimizerRprop.cpp
  src/OptimizerBFGS.cpp
  src/ProbDataAssocPolicy.cpp
  src/SamplerMetropolisHastings.cpp
  src/SamplerHybridMcmc.cpp
  src/util/ThreadedRangeProcessor.cpp
  src/util/ProblemManager.cpp
  src/OptimizerCallbackManager.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/JacobianContainer.cpp
  test/test_sparse_matrix_functions.cpp
  test/TestProblemManager.cpp
  test/TestLineSearch.cpp
  test/TestOptimizerBase.cpp
  test/TestOptimizer.cpp
  test/TestOptimizer2.cpp
  test/TestOptimizerRprop.cpp
  test/TestOptimizerBFGS.cpp
  test/TestSamplerMcmc.cpp
  test/CallbackTest.cpp
  test/TestOptimizationProblem.cpp
  test/CompressedColumnMatrixTest.cpp
  test/MatrixTestHarness.cpp
  test/DenseMatrixTest.cpp
  test/SparseMatrixTest.cpp
  test/LinearSolverTests.cpp
  test/ErrorTermTests.cpp
  test/ProbDataAssocPolicyTest.cpp
  test/MatrixStackTest.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()

