cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_serialization)

find_package(catkin_simple REQUIRED)
catkin_simple()

#common commands for building c++ executables and libraries
INCLUDE(autogen_cameras.cmake)
#INCLUDE(autogen_frames.cmake)

cs_add_library(${PROJECT_NAME} 
#  ${AUTOGEN_FRAME_CPP_FILES}
  ${AUTOGEN_CAMERA_CPP_FILES}
  src/FrameBaseSerialization.cpp
)

find_package(Boost REQUIRED COMPONENTS serialization) 
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
#catkin_add_gtest(${PROJECT_NAME}_tests
#  test/test_main.cpp
#  test/testMultiFrame.cpp
#)
#target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

cs_install()
cs_export()
