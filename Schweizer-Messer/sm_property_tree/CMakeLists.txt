cmake_minimum_required(VERSION 3.0.2)
project(sm_property_tree)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS sm_common)
include_directories(${catkin_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS system filesystem)

catkin_package(
  INCLUDE_DIRS include  ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sm_common
  DEPENDS
)
add_definitions(-D__STRICT_ANSI__)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

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

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/BoostPropertyTreeImplementation.cpp
    )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()
