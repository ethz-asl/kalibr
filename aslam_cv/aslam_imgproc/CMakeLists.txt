cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_imgproc)

find_package(Boost REQUIRED COMPONENTS system)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_random REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(aslam_time REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(numpy_eigen REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

include_directories(include)

add_library(${PROJECT_NAME} src/UndistorterBase.cpp)
ament_target_dependencies(${PROJECT_NAME} sm_common sm_eigen sm_random sm_boost sm_kinematics sm_opencv sm_logging sm_property_tree aslam_time aslam_cameras opencv2_catkin numpy_eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-deprecated-copy -Wno-maybe-uninitialized)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

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
