cmake_minimum_required(VERSION 2.8.3)
project(aslam_splines)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS system )


cs_add_library(${PROJECT_NAME}
  src/BSplinePoseDesignVariable.cpp
  src/BSplineRSPoseDesignVariable.cpp
  src/BSplineExpressions.cpp
  src/BSplineRSExpressions.cpp
  src/EuclideanBSplineDesignVariable.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
catkin_add_gtest(${PROJECT_NAME}_tests
  test/test_main.cpp
  test/TestBSplineExpressions.cpp
  test/TestOPTBSpline.cpp
  test/TestErrors.cpp
)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

cs_install()
cs_export()
