cmake_minimum_required(VERSION 3.0.2)
project(aslam_cv_python)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED COMPONENTS serialization) 
find_package(catkin REQUIRED COMPONENTS
  sm_common
  sm_eigen
  sm_kinematics
  sm_python
  sm_opencv
  opencv2_catkin
  python_module
  aslam_cv_serialization
  aslam_imgproc
  aslam_cameras
  aslam_time
  aslam_cv_serialization
)

catkin_package(
  INCLUDE_DIRS include
)

add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

if(APPLE)
  add_definitions( -ftemplate-depth-1024)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

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
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy") # TODO: Fix...
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)