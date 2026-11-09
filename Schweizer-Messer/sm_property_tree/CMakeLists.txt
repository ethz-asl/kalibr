cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_property_tree)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
include_directories()
find_package(Boost REQUIRED COMPONENTS system filesystem)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sm_common)
add_definitions(-D__STRICT_ANSI__)

include_directories(include ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/PropertyTree.cpp
  src/BoostPropertyTreeImplementation.cpp
  src/BoostPropertyTree.cpp
  src/PropertyTreeImplementation.cpp
  src/BoostPropertyTreeSupport.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_common)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} pthread)

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

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/BoostPropertyTreeImplementation.cpp
    )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()
ament_package()
