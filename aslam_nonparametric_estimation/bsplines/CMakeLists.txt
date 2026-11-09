cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(bsplines)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sparse_block_matrix REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

include_directories(include)

add_library(${PROJECT_NAME} 
  src/BSpline.cpp
  src/BSplinePose.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_common sm_timing sm_eigen sm_kinematics sparse_block_matrix)
target_link_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
ament_package()
