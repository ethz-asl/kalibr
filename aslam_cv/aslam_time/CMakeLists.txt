cmake_minimum_required(VERSION 2.8.3)
project(aslam_time)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS serialization system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


cs_add_library(${PROJECT_NAME}
    src/time.cpp
    src/duration.cpp)

set(OTHER_LIBS)
if(NOT APPLE)
  set(OTHER_LIBS rt)
endif()

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OTHER_LIBS})

cs_install()

cs_export()
