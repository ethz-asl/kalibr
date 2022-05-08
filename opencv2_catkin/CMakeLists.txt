cmake_minimum_required(VERSION 3.0.2)
project(opencv2_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()


cs_install()
cs_export(CFG_EXTRAS opencv2-extras.cmake)
