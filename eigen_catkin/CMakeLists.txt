cmake_minimum_required(VERSION 2.8.3)
project(eigen_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

set(EIGEN_CATKIN_EIGEN_INCLUDE ${CATKIN_DEVEL_PREFIX}/include/eigen3)
set(EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE ${EIGEN_CATKIN_EIGEN_INCLUDE}/signature_of_eigen3_matrix_library)

set(USE_SYSTEM_EIGEN "OFF" CACHE INTERNAL "Use a system installation of Eigen")

include(${CMAKE_CURRENT_SOURCE_DIR}/use-system-installation-of-eigen.cmake)


if(USE_SYSTEM_EIGEN)
  message(STATUS "eigen_catkin is configured to USE system eigen")
  if(EXISTS ${EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE})
    file(RENAME ${EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE} ${EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE}.off)

    if(EIGEN3_INCLUDE_DIR EQUAL EIGEN_CATKIN_EIGEN_INCLUDE)
      unset(EIGEN3_INCLUDE_DIR CACHE)
    endif()
    if(Eigen_INCLUDE_DIRS EQUAL EIGEN_CATKIN_EIGEN_INCLUDE)
      unset(Eigen_INCLUDE_DIRS CACHE)
    endif()
  endif()
  
  # see https://github.com/ethz-asl/ethzasl_icp_mapping/issues/38 for the discussion behind the following
  find_package(Eigen3)

  # Eigen3 will not be found on Ubuntu 12.04, so we moke it
  if(EIGEN3_FOUND)
    set(EIGEN_CATKIN_EIGEN_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE INTERNAL "")
    set(EIGEN_NAME "EIGEN3")
  else()
    # Fallback to cmake_modules
    find_package(Eigen REQUIRED)
    if(EIGEN_FOUND)
      set(EIGEN_CATKIN_EIGEN_INCLUDE ${Eigen_INCLUDE_DIRS} CACHE INTERNAL "")
      set(EIGEN_NAME "Eigen")
    endif()
  endif()

  if(EIGEN_NAME)
    # this way ${EIGEN_NAME} will end up in the cs_export call's DEPENDS argument.
    list(APPEND _DEPENDS ${EIGEN_NAME}) 
    message(STATUS "Found and using system Eigen in ${EIGEN_CATKIN_EIGEN_INCLUDE}")
  else()
    message(FATAL_ERROR "Could not find a system Eigen installation! What about turning on eigen_catkin's own Eigen installation by removing USE_SYSTEM_EIGEN.cmake or switching back to the master branch.")
  endif()
else()
  message(STATUS "eigen_catkin is configured NOT to use system eigen")
  file(MAKE_DIRECTORY ${EIGEN_CATKIN_EIGEN_INCLUDE})
  ExternalProject_Add(eigen_src
    UPDATE_COMMAND ""
    URL https://github.com/ethz-asl/thirdparty_library_binaries/raw/master/Eigen_3.2.7.tar.gz
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}  -DCMAKE_BUILD_TYPE:STRING=Release
  )

  if(EXISTS ${EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE}.off)
    file(RENAME ${EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE}.off ${EIGEN_CATKIN_EIGEN_INCLUDE_SIGNATURE})
  endif()

  message(STATUS "Using eigen_catkin's Eigen in ${EIGEN_CATKIN_EIGEN_INCLUDE}")
endif()

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)

if(NOT USE_SYSTEM_EIGEN)
  add_dependencies(${PROJECT_NAME} eigen_src)
endif()

cs_install()

if(NOT USE_SYSTEM_EIGEN)
  install(DIRECTORY ${EIGEN_CATKIN_EIGEN_INCLUDE}
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
endif()

cs_export(INCLUDE_DIRS ${EIGEN_CATKIN_EIGEN_INCLUDE} 
          DEPENDS ${_DEPENDS}
          CFG_EXTRAS eigen-extras.cmake)
