cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_cameras)

ADD_DEFINITIONS(-DASLAM_USE_ROS)

find_package(ament_cmake REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_logging REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(sm_random REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem)
find_package(Eigen3 REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

include_directories(include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
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
ament_target_dependencies(${PROJECT_NAME} sm_common sm_eigen sm_boost sm_kinematics sm_logging aslam_time sm_opencv sm_property_tree opencv2_catkin sm_random)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy") # TODO: Fix...

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

  ament_add_gtest(${PROJECT_NAME}_tests
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

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
endif()

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
  DESTINATION include
)
ament_package()
