cmake_minimum_required(VERSION 2.8.3)
project(aslam_splines_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_python_export_library(${PROJECT_NAME} python/aslam_splines
  src/spline_module.cpp
  src/BSplineMotionError.cpp
  src/SimpleSplineError.cpp
  src/OPTBSpline.cpp
  src/QuadraticIntegralError.cpp
)

catkin_add_nosetests( 
  test/OptBSpline.py
)
catkin_add_nosetests( 
  test/QuadraticIntegralError.py
)

cs_install()
cs_export()
