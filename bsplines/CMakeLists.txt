cmake_minimum_required(VERSION 2.8.3)
project(bsplines)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)



cs_add_library(${PROJECT_NAME} 
  src/BSpline.cpp
  src/BSplinePose.cpp
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_tests 
  test/SplineTests.cpp
  test/BSplinePoseTests.cpp
  test/DiffManifoldBSplineTests.cpp
  test/test_main.cpp
  )

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})


#--------------------------#
# Documentation
#TODO: have catkin auto generate doc

cs_install()
cs_export()
