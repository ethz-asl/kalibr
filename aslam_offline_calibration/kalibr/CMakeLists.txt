cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(kalibr)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(aslam_cv_python REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(incremental_calibration_python REQUIRED)
find_package(aslam_cameras_april REQUIRED)
find_package(aslam_splines_python REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(python_module REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_python REQUIRED)
find_package(aslam_cv_backend_python REQUIRED)

ament_export_include_directories(include)

##################################
# error terms (+python export)
##################################
include_directories(include)

add_library(${PROJECT_NAME}_errorterms
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)
ament_target_dependencies(${PROJECT_NAME}_errorterms aslam_cv_python aslam_backend aslam_backend_expressions aslam_backend_python incremental_calibration_python aslam_cameras_april aslam_splines_python numpy_eigen python_module sparse_block_matrix sm_python aslam_cv_backend_python)
target_link_libraries(${PROJECT_NAME}_errorterms)

# NB: The ".." is a hack to install the Python bindings to a global location to
# then be able to import it into a local context project. This is required to
# work-around having multiple Python packages in one catkin package and the way
# the PYTHON_PATH overloads work.
add_python_export_library(${PROJECT_NAME}_errorterms_python python/${PROJECT_NAME}_errorterms/..
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_errorterms_python ${PROJECT_NAME}_errorterms)

##################################
# tests
##################################
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/TestErrorTerms.cpp
    )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_errorterms)
endif()

##################################
# EXPORT
##################################
install(TARGETS ${PROJECT_NAME}_errorterms
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(PROGRAMS
    python/kalibr_bagcreater
    python/kalibr_camera_focus
    python/kalibr_bagextractor
    python/kalibr_camera_validator
    python/kalibr_visualize_calibration
    python/kalibr_calibrate_cameras
    python/kalibr_visualize_distortion
    python/kalibr_calibrate_imu_camera
    python/kalibr_create_target_pdf
    python/kalibr_calibrate_rs_cameras
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(kalibr_errorterms PACKAGE_DIR python/kalibr_errorterms)
ament_python_install_package(kalibr_common PACKAGE_DIR python/kalibr_common)
ament_python_install_package(kalibr_camera_calibration PACKAGE_DIR python/kalibr_camera_calibration)
ament_python_install_package(kalibr_imu_camera_calibration PACKAGE_DIR python/kalibr_imu_camera_calibration)
ament_python_install_package(kalibr_rs_camera_calibration PACKAGE_DIR python/kalibr_rs_camera_calibration)

ament_package()
