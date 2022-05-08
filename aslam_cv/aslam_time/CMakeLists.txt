cmake_minimum_required(VERSION 3.0.2)
project(aslam_time)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  LIBRARIES ${PROJECT_NAME}  
  INCLUDE_DIRS include
  DEPENDS Boost
)


include_directories(include ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/time.cpp
  src/duration.cpp
)

set(OTHER_LIBS)
if(NOT APPLE)
  set(OTHER_LIBS rt)
endif()

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OTHER_LIBS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
