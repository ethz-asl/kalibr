cmake_minimum_required(VERSION 3.0.2)
project(aslam_cameras_april)

ADD_DEFINITIONS(-DASLAM_USE_ROS)

find_package(catkin REQUIRED COMPONENTS
  sm_python
  aslam_cameras
  ethz_apriltag2
  numpy_eigen
  sm_common
  sm_logging
  sm_boost
  sm_eigen
  python_module
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS OpenCV Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/GridCalibrationTargetAprilgrid.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_python_export_library(${PROJECT_NAME}_python python/aslam_cameras_april
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_python ${PROJECT_NAME})

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
