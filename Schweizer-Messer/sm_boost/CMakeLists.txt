cmake_minimum_required(VERSION 2.8.3)
project(sm_boost)

find_package(catkin REQUIRED COMPONENTS sm_common)
find_package(Boost REQUIRED COMPONENTS system serialization thread)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES sm_boost
  CATKIN_DEPENDS sm_common
  DEPENDS
)

include_directories(include ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/JobQueue.cpp
  src/portable_binary_iarchive.cpp
  src/portable_binary_oarchive.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
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
    test/testFuture.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()

endif()
