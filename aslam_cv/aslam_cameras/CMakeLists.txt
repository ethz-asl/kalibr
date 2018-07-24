cmake_minimum_required(VERSION 2.8.3)
project(aslam_cameras)

find_package(catkin_simple REQUIRED)

catkin_simple()

ADD_DEFINITIONS(-DASLAM_USE_ROS )

find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

#common commands for building c++ executables and libraries
cs_add_library(${PROJECT_NAME} 
  src/CameraGeometryBase.cpp
  src/GlobalShutter.cpp
  src/RollingShutter.cpp
  src/NoMask.cpp
  src/NoDistortion.cpp
  src/RadialTangentialDistortion.cpp
  src/EquidistantDistortion.cpp
  src/FovDistortion.cpp
  src/ImageMask.cpp
  src/GridCalibrationTargetObservation.cpp
  src/GridCalibrationTargetBase.cpp
  src/GridCalibrationTargetCheckerboard.cpp
  src/GridCalibrationTargetCirclegrid.cpp
  src/GridDetector.cpp
  src/Triangulation.cpp
  src/FrameBase.cpp
  src/KeypointBase.cpp
  src/DescriptorBase.cpp
  src/BackProjection.cpp
  src/Landmark.cpp
  src/Image.cpp
  )

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)


  catkin_add_gtest(${PROJECT_NAME}_tests
    test/test_main.cpp
    test/PinholeCameraGeometry.cpp
    test/OmniCameraGeometry.cpp
    test/ExtendedUnifiedCameraGeometry.cpp
    test/DoubleSphereCameraGeometry.cpp
    test/RadialTangentialDistortion.cpp
    test/EquidistantDistortion.cpp
    test/FovDistortion.cpp
    test/Triangulation.cpp
    test/testFrame.cpp
    test/GridCalibration.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
    )


  SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME} ${Boost_LIBRARIES})

endif()

cs_install()
cs_export()
