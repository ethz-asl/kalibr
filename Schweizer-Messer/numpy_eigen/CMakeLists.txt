cmake_minimum_required(VERSION 2.8.3)
project(numpy_eigen)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${Boost_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/include/numpy_eigen")
add_definitions(-std=c++0x -D__STRICT_ANSI__)

INCLUDE(src/autogen_files.cmake)
add_python_export_library(${PROJECT_NAME} 
  ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_FILES}
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

INCLUDE(src/autogen_test_files.cmake)
add_python_export_library(${PROJECT_NAME}_test 
      ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_TEST_FILES}
  )

## Add nosetest based cpp test target.
catkin_add_nosetests(test/numpy_eigen_tests.py)

cs_install()
cs_export()
