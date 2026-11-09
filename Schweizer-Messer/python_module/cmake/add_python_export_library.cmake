# Helpful function for adding python export libraries (boost::python modules)
# in ROS 2.  This is the ament_cmake port of the original catkin macro; the
# semantics are preserved: a shared library target is created from the given
# sources and installed into the Python package directory so that it can be
# imported next to the package's __init__.py.
#
# Usage:
#   find_package(python_module REQUIRED)
#   add_python_export_library(${PROJECT_NAME}_python python/${PROJECT_NAME}
#                             src/module.cpp src/file1.cpp)
#
# The second argument is the directory of the python package.  Its leaf is used
# as the name of the package directory the extension module is installed into.
# A trailing "/.." is supported (historical kalibr hack) and installs the
# extension module into the top-level site-packages directory instead.

FUNCTION(add_python_export_library TARGET_NAME PYTHON_MODULE_DIRECTORY)

  # Get the leaf of the python module directory.  This is the python package name.
  get_filename_component(TMP "${PYTHON_MODULE_DIRECTORY}/garbage.txt" PATH)
  get_filename_component(PYTHON_PACKAGE_NAME "${TMP}.txt" NAME_WE)

  # Find Python
  find_package(PythonLibs REQUIRED)
  include_directories(${PYTHON_INCLUDE_DIRS})

  # Determine the boost::python component name.
  if(APPLE)
    set(BOOST_COMPONENTS system)
  else()
    set(BOOST_COMPONENTS)
  endif()
  if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
    find_package(Boost QUIET)
    if(Boost_VERSION LESS 106700)
      list(APPEND BOOST_COMPONENTS python)
    else()
      # The boost_python library has been renamed in Boost 1.67 and the
      # FindBoost.cmake module requires a Python version suffix.
      list(APPEND BOOST_COMPONENTS python27)
    endif()
  else()
    # Derive the boost_python component name from the Python version found
    # by FindPythonLibs.  Boost names its python libraries with the major and
    # minor version concatenated, e.g. python310 for Python 3.10.
    if(PYTHONLIBS_VERSION_STRING MATCHES "^([0-9]+)\\.([0-9]+)")
      list(APPEND BOOST_COMPONENTS "python${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
    else()
      list(APPEND BOOST_COMPONENTS python38)
    endif()
  endif()
  find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})

  # Create the target and assign source files
  add_library(${TARGET_NAME} ${ARGN})

  # Link the python module against the Python and boost::python libraries.
  target_link_libraries(${TARGET_NAME}
    ${PYTHON_LIBRARY}
    ${Boost_LIBRARIES}
    )

  # Install the extension module next to the python package files.
  # Ensure the python install dir is defined (the calling package must have
  # found ament_cmake_python before invoking this function).
  if(NOT DEFINED PYTHON_INSTALL_DIR)
    ament_get_python_install_dir(_APEL_IGNORED)
  endif()
  if("${PYTHON_MODULE_DIRECTORY}" MATCHES "/\\.\\.$")
    # Historical hack: install to the global python path.
    set(_APEL_INSTALL_DIR ${PYTHON_INSTALL_DIR})
  else()
    set(_APEL_INSTALL_DIR ${PYTHON_INSTALL_DIR}/${PYTHON_PACKAGE_NAME})
  endif()
  install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION ${_APEL_INSTALL_DIR}
    LIBRARY DESTINATION ${_APEL_INSTALL_DIR}
  )

ENDFUNCTION()

