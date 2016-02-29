cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_backend)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/GridCalibrationTargetDesignVariableContainer.cpp
  # src/CameraGeometryDesignVariableContainer.cpp
  # src/NCameraSystemDesignVariableContainer.cpp
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
)

find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()

cs_export()
