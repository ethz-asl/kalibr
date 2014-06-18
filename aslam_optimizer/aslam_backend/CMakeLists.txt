cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${Eigen_INCLUDE_DIRS})

#add_definitions( -fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x -O3)

#target_link_libraries(${PROJECT_NAME} ${CHOLMOD_LIBRARIES} ${EXTRA_LIBS}  ${CSPARSE_LIBRARY})
#rosbuild_link_boost(${PROJECT_NAME} thread system)

cs_add_library(${PROJECT_NAME}
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

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
catkin_add_gtest(${PROJECT_NAME}_test
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

cs_install()
cs_export()

