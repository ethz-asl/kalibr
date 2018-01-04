cmake_minimum_required(VERSION 2.8.3)
project(aslam_splines)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system )


cs_add_library(${PROJECT_NAME}
  src/BSplinePoseDesignVariable.cpp
  src/BSplineExpressions.cpp
  src/EuclideanBSplineDesignVariable.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/TestBSplineExpressions.cpp
    test/TestErrors.cpp
    #  test/TestOPTBSpline.cpp
    )

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

endif()

cs_install()
cs_export()
