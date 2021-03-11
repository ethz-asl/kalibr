cmake_minimum_required(VERSION 2.8.3)
project(aslam_imgproc)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Boost REQUIRED COMPONENTS system)

include_directories(${EIGEN3_INCLUDE_DIR})
cs_add_library(${PROJECT_NAME} src/UndistorterBase.cpp)


target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

cs_install()
cs_export()
