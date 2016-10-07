cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend_bsplines_tutorial)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/ErrorTermMotionBST.cpp
  src/ErrorTermPriorBST.cpp
  src/exampleBST.cpp
)

find_package(Boost REQUIRED COMPONENTS system) 
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


cs_add_executable(${PROJECT_NAME}_exec
  src/exampleBST.cpp
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)


catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/TestErrorTestMotionBST.cpp
)

target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
