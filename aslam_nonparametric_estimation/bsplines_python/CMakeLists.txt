cmake_minimum_required(VERSION 3.0.2)
project(bsplines_python)

find_package(catkin_simple REQUIRED)
catkin_simple()

# Set up the python exports.
SET(PY_PROJECT_NAME bsplines_python)
SET(PY_PACKAGE_DIR python/bsplines)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/SplinePython.cpp
  src/BSplinePython.cpp
  src/BSplinePosePython.cpp
)

cs_install()
cs_export()


