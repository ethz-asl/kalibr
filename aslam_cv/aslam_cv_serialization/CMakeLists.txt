
cmake_minimum_required(VERSION 3.0.2)
project(aslam_cv_serialization)

find_package(catkin REQUIRED COMPONENTS
  aslam_cameras
  aslam_time
  sm_common
  sm_boost
  sm_property_tree
  sm_eigen
  sm_kinematics
  sm_opencv
  sm_logging
  opencv2_catkin
  sm_random
)
find_package(Boost REQUIRED COMPONENTS serialization system)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

#common commands for building c++ executables and libraries
INCLUDE(autogen_cameras.cmake)
#INCLUDE(autogen_frames.cmake)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  ${AUTOGEN_CAMERA_CPP_FILES}
  src/FrameBaseSerialization.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
#catkin_add_gtest(${PROJECT_NAME}_tests
#  test/test_main.cpp
#  test/testMultiFrame.cpp
#)
#target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

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
