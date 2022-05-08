cmake_minimum_required(VERSION 3.0.2)
project(aslam_cv_backend)

find_package(catkin REQUIRED COMPONENTS
  sm_common
  sm_boost
  sm_kinematics
  sm_eigen
  sm_opencv
  sm_property_tree
  sm_logging
  sm_timing
  aslam_backend
  aslam_backend_expressions
  aslam_time
  aslam_cameras
  opencv2_catkin
  sparse_block_matrix
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
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

##################
## Installation ##
##################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
