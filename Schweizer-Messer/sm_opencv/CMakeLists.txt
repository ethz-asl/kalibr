cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_opencv)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_export_include_directories(include)
ament_export_dependencies(sm_common)
add_definitions(-D__STRICT_ANSI__)

install(DIRECTORY include/
  DESTINATION include
)
ament_package()
