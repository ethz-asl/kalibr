cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(sparse_block_matrix)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(SuiteSparse REQUIRED)

find_package(Eigen3)

include_directories(
  include
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_export_include_directories(include/)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(sm_common sm_eigen)

add_library(${PROJECT_NAME}
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_common sm_eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ament_add_gtest( ${PROJECT_NAME}_tests
    test/test_main.cpp
    test/solver_tests.cpp
    test/sparse_block_matrix_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} ${TBB_LIBRARIES} ${SUITESPARSE_LIBRARIES})
endif()

# Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(FILES cmake/FindSuiteSparse.cmake DESTINATION share/${PROJECT_NAME}/cmake/)
ament_package(CONFIG_EXTRAS cmake/FindSuiteSparse.cmake cmake/sparse_block_matrix-extras.cmake.in)
