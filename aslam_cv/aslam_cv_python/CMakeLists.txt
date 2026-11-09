cmake_minimum_required(VERSION 3.8)
set(BUILD_SHARED_LIBS ON)

project(aslam_cv_python)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED COMPONENTS serialization) 
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_python REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(opencv2_catkin REQUIRED)
find_package(python_module REQUIRED)
find_package(aslam_cv_serialization REQUIRED)
find_package(aslam_imgproc REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_time REQUIRED)
find_package(aslam_cv_serialization REQUIRED)

ament_export_include_directories(include)

add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

if(APPLE)
  add_definitions( -ftemplate-depth-1024)
endif()

include_directories(include)

add_python_export_library(${PROJECT_NAME} python/aslam_cv
  src/module.cpp
  src/CameraGeometries.cpp
  src/CameraGeometry.cpp
  src/Time.cpp
#  src/CameraSystem.cpp
#  src/CameraSystemClasses.cpp
#  src/MatchingAlgorithms.cpp
#  src/MultiFrame.cpp
  src/Frontend.cpp
  src/Frame.cpp
#  src/Descriptors.cpp
  src/CameraProjections.cpp
  src/CameraShutters.cpp
#  src/ImageSynchronizer.cpp
#  src/FrameBuilder.cpp
#  src/ImageContainer.cpp
  src/GridCalibration.cpp
#  src/Landmark.cpp
  src/Undistorters.cpp
#  src/NCameras.cpp
#  src/MultiFrameFactory.cpp
  src/OmniUndistorter.cpp
  src/PinholeUndistorter.cpp
)
ament_target_dependencies(${PROJECT_NAME} sm_common sm_eigen sm_kinematics sm_python sm_opencv opencv2_catkin python_module aslam_cv_serialization aslam_imgproc aslam_cameras aslam_time)
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy") # TODO: Fix...
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION include
)
ament_python_install_package(aslam_cv PACKAGE_DIR python/aslam_cv)

ament_package()
