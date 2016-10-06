cmake_minimum_required(VERSION 2.8.3)
project(python_module)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS add_python_export_library.cmake
)