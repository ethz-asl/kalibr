cmake_minimum_required(VERSION 2.8.3)
project(sm_matlab)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${Boost_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Matlab)  # Don't search with required, so we can continue.

if(MATLAB_FOUND)
  cs_add_library(${PROJECT_NAME}
    src/Engine.cpp
  )
  include_directories(${MATLAB_INCLUDE_DIR})
  
  target_link_libraries(${PROJECT_NAME} ${MATLAB_LIBRARIES}
                                        ${Boost_LIBRARIES})

  add_executable(sm_matlab_demo demo/demo.cpp)
  target_link_libraries(sm_matlab_demo 
                        ${PROJECT_NAME})

  #############
  ## Testing ##
  #############
# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/MatlabInterfaceTests.cpp
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
else()
message("Matlab not found, not building sm_matlab")  
endif()

cs_install()
cs_export()
