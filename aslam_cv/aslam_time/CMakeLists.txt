cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_time)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
ament_package()
