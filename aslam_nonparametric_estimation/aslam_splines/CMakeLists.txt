cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_splines)

find_package(ament_cmake REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(bsplines REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(bsplines)

include_directories(include)

add_library(${PROJECT_NAME}
  src/BSplinePoseDesignVariable.cpp
  src/BSplineExpressions.cpp
  src/EuclideanBSplineDesignVariable.cpp
)
ament_target_dependencies(${PROJECT_NAME} aslam_backend aslam_backend_expressions bsplines numpy_eigen sparse_block_matrix sm_common sm_kinematics sm_timing opencv2_catkin)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/TestBSplineExpressions.cpp
    test/TestErrors.cpp
    #  test/TestOPTBSpline.cpp
    )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

##################
## Installation ##
##################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)
ament_package()
