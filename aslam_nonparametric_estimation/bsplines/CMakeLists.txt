cmake_minimum_required(VERSION 3.0.2)
project(bsplines)

find_package(catkin REQUIRED COMPONENTS
  sm_common
  sm_timing
  sm_eigen
  sm_kinematics
  sparse_block_matrix
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/BSpline.cpp
  src/BSplinePose.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
