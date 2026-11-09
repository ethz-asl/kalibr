cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_cameras_april)

ADD_DEFINITIONS(-DASLAM_USE_ROS)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sm_python REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(ethz_apriltag2 REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(python_module REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system)
find_package(Eigen3 REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

include_directories(include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/GridCalibrationTargetAprilgrid.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_python aslam_cameras ethz_apriltag2 numpy_eigen sm_common sm_logging sm_boost sm_eigen python_module)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_python_export_library(${PROJECT_NAME}_python python/aslam_cameras_april
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_python ${PROJECT_NAME})

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
ament_python_install_package(aslam_cameras_april PACKAGE_DIR python/aslam_cameras_april)

ament_package()
