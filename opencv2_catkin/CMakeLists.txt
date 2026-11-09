cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(opencv2_catkin)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

set(opencv2_catkin_LIBRARY_DIRS ${OpenCV_LIB_DIR})
set(opencv2_catkin_LIBRARIES ${OpenCV_LIBS})
set(opencv2_catkin_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

ament_export_include_directories(${OpenCV_INCLUDE_DIRS})
ament_export_libraries(${OpenCV_LIBS})

ament_package()
