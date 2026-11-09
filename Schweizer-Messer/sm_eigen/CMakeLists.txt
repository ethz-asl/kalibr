cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sm_eigen)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_random REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sm_common sm_random)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}  
  src/random.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_common sm_random)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

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

  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/EigenSerializationTests.cpp
    test/MatrixSqrtTest.cpp
    )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()
ament_package(CONFIG_EXTRAS cmake/sm_eigen-extras.cmake.in)
