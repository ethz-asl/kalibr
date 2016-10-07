cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend_python)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions( -fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x)


# This functions take TARGET_NAME PYTHON_MODULE_DIRECTORY sourceFile1 [sourceFile2 ...]
add_python_export_library(${PROJECT_NAME} python/aslam_backend
  src/module.cpp
  src/Backend.cpp
  src/Optimizer.cpp
  src/OptimizerOptions.cpp
  src/OptimizationProblem.cpp
  src/DesignVariable.cpp
  src/ErrorTerm.cpp
  src/ErrorTermDs.cpp
  src/ScalarNonSquaredErrorTerm.cpp
  src/MEstimators.cpp
  src/JacobianContainer.cpp
  src/BackendExpressions.cpp
  src/CompressedColumnMatrix.cpp
  src/LinearSystemSolver.cpp
  src/ErrorTermTransformation.cpp
  src/L1Regularizer.cpp
  src/TrustRegionPolicies.cpp
  src/SparseBlockMatrix.cpp
  src/DesignVariableTimePair.cpp
  src/SampleDvAndError.cpp
  src/Sampler.cpp
)

find_package(Boost REQUIRED COMPONENTS thread) 
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

cs_install()
cs_export()

catkin_add_nosetests( 
  test/Test.py
  DEPENDENCIES ${PROJECT_NAME}
)
