cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_common)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D__STRICT_ANSI__")


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
  
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/progress_info.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy") # TODO: Fix...

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
  DESTINATION include #include
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ament_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/maths.cpp
    test/serialization_macros.cpp
    test/numerical_comparisons.cpp
    test/hash_id.cpp
    )
  target_link_libraries(${PROJECT_NAME}-test pthread)
endif()
ament_package(CONFIG_EXTRAS cmake/export_flags.cmake)
