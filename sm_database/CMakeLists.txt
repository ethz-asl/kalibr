cmake_minimum_required(VERSION 2.8.3)
project(sm_database)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(Sqlite3 REQUIRED)
include_directories(${SQLITE3_DIRECTORIES})

include_directories(include ${Boost_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME}
  src/Database.cpp
)

target_link_libraries(${PROJECT_NAME} ${SQLITE3_LIBRARIES}
  ${Boost_LIBRARIES})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test   
  test/test_main.cpp
  test/SerializedMap.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} 
                                             ${SQLITE3_LIBRARIES}
                                             ${catkin_LIBRARIES})
endif()

cs_install()
cs_export()
