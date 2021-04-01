cmake_minimum_required(VERSION 3.0.2)
project(aslam_cv_error_terms)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  sm_logging
  sm_property_tree
  sm_opencv
  sm_kinematics
  aslam_time
  aslam_cameras
  aslam_backend
  aslam_cv_backend
  aslam_backend_expressions
  sm_boost
  sm_eigen
  sm_timing
  sparse_block_matrix
  opencv2_catkin
  aslam_splines
)

catkin_package(
  INCLUDE_DIRS include
)

if(CATKIN_ENABLE_TESTING)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  include_directories(include ${catkin_INCLUDE_DIRS})
  # catkin_add_gtest(${PROJECT_NAME}_test
  #   test/test_main.cpp
  #   test/TestReprojectionError.cpp
  # )
  # target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
