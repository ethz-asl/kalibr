# Helpful function for adding python export libraries in ROS.
# Usage:
# 
# rosbuild_find_ros_package(numpy_eigen)
# include(${numpy_eigen_PACKAGE_PATH}/cmake/add_python_export_library.cmake)
# add_python_export_library(${PROJECT_NAME}_python ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}
#                           src/file1.cpp
#                           src/file2.cpp
#                          )
#
#
# Set the path for the output python files. This should be the path
# with the __init__.py file. The standard for ROS (where python message
# definitions live) is ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}


FUNCTION(add_python_export_library TARGET_NAME PYTHON_MODULE_DIRECTORY )

  # Cmake is a very bad scripting language. Very bad indeed.
  # Get the leaf of the python module directory. This is the python package name
  # This first command makes sure to strip off the trailing /
  get_filename_component(TMP "${PYTHON_MODULE_DIRECTORY}/garbage.txt" PATH)
  # This grabs the leaf of the path
  get_filename_component(PYTHON_PACKAGE_NAME "${TMP}.txt" NAME_WE)
  # This grabs the parent of the leaf
  get_filename_component(PYTHON_MODULE_DIRECTORY_PREFIX "${TMP}.txt" PATH)


  set(SETUP_PY "${CMAKE_CURRENT_SOURCE_DIR}/setup.py")
  if(EXISTS ${SETUP_PY}) 
  else()
    set(SETUP_PY_TEXT "
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['${PYTHON_PACKAGE_NAME}'],
    package_dir={'':'${PYTHON_MODULE_DIRECTORY_PREFIX}'})

setup(**setup_args)
")

    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/setup.py" ${SETUP_PY_TEXT})
    message( SEND_ERROR "Error! ${SETUP_PY} does not exist
This is a problem. Let me tell you about it.

In the rosbuild days, we used the manifest.xml file to export python libs and __init__.py files. With catkin, this is no longer possible. Now we have to include a setup.py file that tells catkin what directories we need installed. The setup.py file should be in the root of your package source directory.

From how you called this function, I expect that your package looks like this:

${CMAKE_CURRENT_SOURCE_DIR}/${PYTHON_MODULE_DIRECTORY_PREFIX}/${PYTHON_PACKAGE_NAME}/__init__.py

Then your setup.py file should look like this. I have done the variable substitution for you. I'm now writing this text into ${SETUP_PY}. Please commit this to your source repository.

${SETUP_PY_TEXT}

")
  endif()
  # Force the user to have a setup.py file
  catkin_python_setup()

  # Find Python
  FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})

  if(APPLE)
    SET(BOOST_COMPONENTS system)
  else()
    SET(BOOST_COMPONENTS)
  endif()
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost QUIET)
    if(Boost_VERSION LESS 106700)
      list(APPEND BOOST_COMPONENTS python)
    else()
      # The boost_python library has been renamed in Boost 1.67 and the FindBoost.cmake
      # module requires a Python version suffix:
      #
      # References:
      # - https://www.boost.org/docs/libs/1_67_0/libs/python/doc/html/rn.html
      # - https://cmake.org/cmake/help/v3.12/module/FindBoost.html
      #
      list(APPEND BOOST_COMPONENTS python27)
    endif()
  else()
    list(APPEND BOOST_COMPONENTS python3)
  endif()
  find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS}) 

  IF(APPLE)
    # The apple framework headers don't include the numpy headers for some reason.
    GET_FILENAME_COMPONENT(REAL_PYTHON_INCLUDE ${PYTHON_INCLUDE_DIRS} REALPATH)
    IF( ${REAL_PYTHON_INCLUDE} MATCHES Python.framework)
      message("Trying to find extra headers for numpy from ${REAL_PYTHON_INCLUDE}.")
      message("Looking in ${REAL_PYTHON_INCLUDE}/../../Extras/lib/python/numpy/core/include/numpy")
      FIND_PATH(NUMPY_INCLUDE_DIR arrayobject.h
	${REAL_PYTHON_INCLUDE}/../../Extras/lib/python/numpy/core/include/numpy
	${REAL_PYTHON_INCLUDE}/numpy
	)
      IF(${NUMPY_INCLUDE_DIR} MATCHES NOTFOUND)
	message("Unable to find numpy include directories: ${NUMPY_INCLUDE_DIR}")
      ELSE()
	message("Found headers at ${NUMPY_INCLUDE_DIR}")
	INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR}/..)
      ENDIF()
    ENDIF()
  ENDIF(APPLE)


  # message("Target files: ${ARGN}")
  # Create the target and assign source files
  add_library( ${TARGET_NAME}
      ${ARGN}
    )

  # Link your python project to the main library and to Python
  target_link_libraries( ${TARGET_NAME}
    ${PYTHON_LIBRARY}
    ${catkin_LIBRARIES}
    )

  # Link against boost::python
  target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES})

  # On OSX and Linux, the python library must end in the extension .so. Build this
  # filename here.
  get_property(PYLIB_OUTPUT_FILE TARGET ${TARGET_NAME} PROPERTY LOCATION)
  get_filename_component(PYLIB_OUTPUT_NAME ${PYLIB_OUTPUT_FILE} NAME_WE)
  set(PYLIB_SO_NAME ${PYLIB_OUTPUT_NAME}.so)

  if(APPLE)
    SET(DIST_DIR site-packages)
  else()
    SET(DIST_DIR dist-packages)
  endif()

  install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/python2.7/${DIST_DIR}/${TARGET_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/python2.7/${DIST_DIR}/${TARGET_NAME}
  )
  
    # Cause the library to be output in the correct directory.
  set(PYTHON_LIB_DIR ${CATKIN_DEVEL_PREFIX}/lib/python2.7/${DIST_DIR}/${PYTHON_PACKAGE_NAME})
  add_custom_command(TARGET ${TARGET_NAME}
    POST_BUILD
    COMMAND mkdir -p ${PYTHON_LIB_DIR} && cp -v ${PYLIB_OUTPUT_FILE} ${PYTHON_LIB_DIR}/${PYLIB_SO_NAME}
    WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}
    COMMENT "Copying library files to python directory" )

  get_directory_property(AMCF ADDITIONAL_MAKE_CLEAN_FILES)
  list(APPEND AMCF ${PYTHON_LIB_DIR}/${PYLIB_SO_NAME})
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${AMCF}") 
  
ENDFUNCTION()

