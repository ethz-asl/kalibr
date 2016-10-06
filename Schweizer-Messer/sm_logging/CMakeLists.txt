cmake_minimum_required(VERSION 2.8.3)
project(sm_logging)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system thread regex)
include_directories(include ${Boost_INCLUDE_DIRS})
add_definitions("-std=c++0x")

cs_add_library(${PROJECT_NAME} 
  src/Logger.cpp
  src/StdOutLogger.cpp
  src/LoggingEvent.cpp
  src/LoggingGlobals.cpp
  src/Formatter.cpp
  src/Tokens.cpp
  src/Levels.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_main.cpp
  test/logTest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    pthread
  )
endif()

cs_install()
cs_export(
  LIBRARIES ${Boost_LIBRARIES}
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)
