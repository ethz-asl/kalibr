cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(ethz_apriltag2)

add_definitions(-fPIC -O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

include_directories(include ${EIGEN3_INCLUDE_DIRS})

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

#demo (requires libv4l2; not needed for calibration, kept optional)
if(NOT APPLE AND EXISTS /usr/include/libv4l2.h)
  add_executable(apriltags_demo src/example/apriltags_demo.cpp src/example/Serial.cpp)
  target_link_libraries(apriltags_demo ${PROJECT_NAME} v4l2)
endif()

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
