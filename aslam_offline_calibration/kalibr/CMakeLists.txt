cmake_minimum_required(VERSION 3.0.2)
project(kalibr)

find_package(catkin_simple REQUIRED)
catkin_simple()

##################################
# error terms (+python export)
##################################
cs_add_library(${PROJECT_NAME}_errorterms
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)

add_python_export_library(${PROJECT_NAME}_errorterms_python python/${PROJECT_NAME}_errorterms/..
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_errorterms_python ${PROJECT_NAME}_errorterms)

##################################
# tests
##################################
if(CATKIN_ENABLE_TESTING)

  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/TestErrorTerms.cpp
    )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_errorterms)

endif()

##################################
# EXPORT
##################################
catkin_python_setup()
cs_install()
cs_export()

catkin_install_python(
  PROGRAMS
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
