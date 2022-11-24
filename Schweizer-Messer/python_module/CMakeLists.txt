cmake_minimum_required(VERSION 3.0.2)
project(python_module)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS add_python_export_library.cmake
)