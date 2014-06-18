cmake_minimum_required(VERSION 2.8.3)
project(foo)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(my_exec src/main.cpp)

cs_install()

cs_export()
