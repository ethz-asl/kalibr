cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_backend_python)

find_package(catkin_simple REQUIRED)

catkin_simple()

add_python_export_library(${PROJECT_NAME} python/aslam_cv_backend
  src/module.cpp  
  src/GridCalibration.cpp
  src/CameraGeometryDesignVariableContainer.cpp
#  src/NCameraSystemDesignVariableContainer.cpp
)

cs_install()

cs_export()
