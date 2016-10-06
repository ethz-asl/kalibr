cmake_minimum_required(VERSION 2.8.3)
project(sm_doc)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  DEPENDS
)

##############
## Building ##
##############

#--------------------------#
# Documentation
SET(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)


FIND_PACKAGE(Doxygen REQUIRED)
IF(DOXYGEN)
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  SET(DOXYGEN_IMAGE_DIRS "\"${DOCSOURCE_DIR}/figs/\"")
  SET(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/../sm_boost/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_common/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_database/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_eigen/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_kinematics/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_opencv/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_pose_graph/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_property_tree/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_python/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_random/include/\" \"${CMAKE_CURRENT_SOURCE_DIR}/../sm_timing/include/\"")

  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  SET(DOXYGEN_QUIET YES)
  MAKE_DIRECTORY(${HTML_DIR})
  SET( HAVE_DOT YES )
  
  CONFIGURE_FILE(${DOCSOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)
  IF(DOXYGEN_VERSION VERSION_GREATER 1.8.0) # This version bound might be wrong. But I could not find the precise version transition when this option vanished - not mentioned in the doxygen changelog at all!
    SET(DOX_FLAGS)
  ELSE()
    SET(DOX_FLAGS --silent)
  ENDIF()
  ADD_CUSTOM_TARGET(doc ALL ${DOXYGEN} ${DOX_FLAGS} ${DOCSOURCE_DIR}/doxygen.config)
ENDIF(DOXYGEN)

#-------------------------#