cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_cv_backend)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_timing REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_time REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

include_directories(include)

add_library(${PROJECT_NAME}
  src/GridCalibrationTargetDesignVariableContainer.cpp
  # src/CameraGeometryDesignVariableContainer.cpp
  # src/NCameraSystemDesignVariableContainer.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_common sm_boost sm_kinematics sm_eigen sm_opencv sm_property_tree sm_logging sm_timing aslam_backend aslam_backend_expressions aslam_time aslam_cameras opencv2_catkin sparse_block_matrix)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

##################
## Installation ##
##################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)
ament_package()
