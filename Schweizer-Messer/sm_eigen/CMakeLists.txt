cmake_minimum_required(VERSION 2.8.3)
project(sm_eigen)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Boost REQUIRED COMPONENTS system serialization)

include_directories(${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

cs_add_library(${PROJECT_NAME}
  src/random.cpp
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
  test/EigenSerializationTests.cpp
  test/MatrixSqrtTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
  )
endif()

cs_install()
cs_export(LIBRARIES ${Boost_LIBRARIES})
