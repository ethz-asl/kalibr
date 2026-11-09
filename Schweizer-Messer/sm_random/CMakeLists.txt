cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_random)

find_package(ament_cmake REQUIRED)
include_directories()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

include_directories(include)

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/random.cpp
)

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
