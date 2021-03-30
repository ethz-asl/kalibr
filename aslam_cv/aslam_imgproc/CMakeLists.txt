cmake_minimum_required(VERSION 3.0.2)
project(aslam_imgproc)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Boost REQUIRED COMPONENTS system)

include_directories(${EIGEN3_INCLUDE_DIRS})
cs_add_library(${PROJECT_NAME} src/UndistorterBase.cpp)
target_compile_options(${PROJECT_NAME} PUBLIC -Wno-deprecated-copy -Wno-maybe-uninitialized)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

cs_install()
cs_export()
