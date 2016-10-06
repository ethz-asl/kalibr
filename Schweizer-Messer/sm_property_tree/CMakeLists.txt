cmake_minimum_required(VERSION 2.8.3)
project(sm_property_tree)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system filesystem)

add_definitions(-D__STRICT_ANSI__)
remove_definitions(-std=c++0x -std-c++11)

include_directories(include ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

cs_add_library(${PROJECT_NAME}
  src/PropertyTree.cpp
  src/BoostPropertyTreeImplementation.cpp
  src/BoostPropertyTree.cpp
  src/PropertyTreeImplementation.cpp
  src/BoostPropertyTreeSupport.cpp
  src/BoostPropertyTreeLoader.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      ${Boost_LIBRARIES})

#############
## Testing ##
#############
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

cs_install()
cs_export(LIBRARIES ${Boost_LIBRARIES})
