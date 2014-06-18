cmake_minimum_required(VERSION 2.8.3)
project(aslam_splines)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system )


cs_add_library(${PROJECT_NAME}
  src/BSplinePoseDesignVariable.cpp
  src/BSplineExpressions.cpp
  src/EuclideanBSplineDesignVariable.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

cs_install()
cs_export()
