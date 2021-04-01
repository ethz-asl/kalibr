cmake_minimum_required(VERSION 3.0.2)
project(aslam_imgproc)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  sm_common
  sm_common
  sm_eigen
  sm_random
  sm_boost
  sm_kinematics
  sm_opencv
  sm_logging
  sm_property_tree
  aslam_time
  aslam_cameras
  opencv2_catkin
  numpy_eigen
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/UndistorterBase.cpp)
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-deprecated-copy -Wno-maybe-uninitialized)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

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
