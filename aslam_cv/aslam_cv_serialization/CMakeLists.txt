
cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_cv_serialization)

find_package(ament_cmake REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_logging REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(sm_random REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

#common commands for building c++ executables and libraries
INCLUDE(autogen_cameras.cmake)
#INCLUDE(autogen_frames.cmake)

include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  ${AUTOGEN_CAMERA_CPP_FILES}
  src/FrameBaseSerialization.cpp
)
ament_target_dependencies(${PROJECT_NAME} aslam_cameras aslam_time sm_common sm_boost sm_property_tree sm_eigen sm_kinematics sm_opencv sm_logging opencv2_catkin sm_random)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
#ament_add_gtest(${PROJECT_NAME}_tests
#  test/test_main.cpp
#  test/testMultiFrame.cpp
#)
#target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

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
