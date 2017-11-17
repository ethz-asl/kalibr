cmake_minimum_required(VERSION 2.8.3)
project(sm_timing)

find_package(catkin REQUIRED COMPONENTS sm_common sm_random)
include_directories(${catkin_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include  ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sm_common sm_random
  DEPENDS
)
add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/Timer.cpp
  src/NsecTimeUtilities.cpp
)
target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)

  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/TestTimestampCorrector.cpp
    test/TestNsecTimeUtilities.cpp
    )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
  endif()

endif()
