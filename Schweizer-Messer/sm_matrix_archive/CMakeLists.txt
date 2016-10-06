cmake_minimum_required(VERSION 2.8.3)
project(sm_matrix_archive)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system filesystem)

add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(include ${Boost_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME}
  src/MatrixArchive.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

cs_add_executable(lsma 
  src/lsma.cpp
)
target_link_libraries(lsma ${PROJECT_NAME})

set_target_properties(lsma
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/bin"
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_main.cpp
  test/TestMatrixArchive.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

cs_install()
cs_export()
