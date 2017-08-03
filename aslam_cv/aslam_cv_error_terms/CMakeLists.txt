cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_error_terms)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system )

if(CATKIN_ENABLE_TESTING)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/TestReprojectionError.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

  target_link_libraries(${PROJECT_NAME}_test ${Boost_LIBRARIES})

endif()

cs_install()
cs_export()
