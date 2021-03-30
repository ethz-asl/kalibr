cmake_minimum_required(VERSION 3.0.2)

project(ethz_apriltag2)

find_package(catkin REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-fPIC -O3)
include_directories(include  ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#demo
if(NOT APPLE)
  add_executable(apriltags_demo src/example/apriltags_demo.cpp src/example/Serial.cpp)
  target_link_libraries(apriltags_demo ${PROJECT_NAME} v4l2)
endif()


