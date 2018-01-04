cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_backend)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/GridCalibrationTargetDesignVariableContainer.cpp
  # src/CameraGeometryDesignVariableContainer.cpp
  # src/NCameraSystemDesignVariableContainer.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    )

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

endif()

cs_install()

cs_export()
