cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_cv_error_terms)

find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(aslam_time REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_cv_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(aslam_splines REQUIRED)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  include_directories(include)
  # ament_add_gtest(${PROJECT_NAME}_test
  #   test/test_main.cpp
  #   test/TestReprojectionError.cpp
  # )
  # target_link_libraries(${PROJECT_NAME}_test ${Boost_LIBRARIES})
endif()

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION include
)
ament_package()
