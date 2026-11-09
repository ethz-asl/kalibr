cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(numpy_eigen)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include/numpy_eigen")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
add_definitions(-D__STRICT_ANSI__)

# ignore "error: return-statement with a value, in function returning ‘void’" in "__multiarray_api.h"
add_compile_options(-fpermissive)

include_directories(include ${Boost_INCLUDE_DIRS})

IF(APPLE)
  link_directories( /usr/local/lib )
ENDIF(APPLE)

##############
## Building ##
##############

INCLUDE(src/autogen_files.cmake)
add_python_export_library(${PROJECT_NAME} 
  ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_FILES}
)
ament_target_dependencies(${PROJECT_NAME} python_module)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
                      
INCLUDE(src/autogen_test_files.cmake)
add_python_export_library(${PROJECT_NAME}_test 
      ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_TEST_FILES}
  )

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ## Add nosetest based cpp test target.
  endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
ament_package(CONFIG_EXTRAS cmake/numpy_eigen-extras.cmake.in)
