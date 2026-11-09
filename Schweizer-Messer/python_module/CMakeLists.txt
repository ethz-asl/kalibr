cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(python_module)

find_package(ament_cmake REQUIRED)

ament_package(CONFIG_EXTRAS cmake/add_python_export_library.cmake)
