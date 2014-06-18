# Import the numpy to Eigen type conversion.
import roslib; roslib.load_manifest('numpy_eigen'); import numpy_eigen
# Import the sm library
import roslib; roslib.load_manifest('sm_python'); import sm
# Import the aslam backend
import roslib; roslib.load_manifest('aslam_backend'); import aslam_backend
import roslib; roslib.load_manifest('aslam_cv_python'); import aslam_cv
# Import the the C++ exports from your package library.
from libaslam_cv_backend_python import *
# Import other files in the directory
# from mypyfile import *

# Now build some convenience wrappers
class CameraModel(object):
    pass

class DistortedOmni(CameraModel):
    geometry = aslam_cv.DistortedOmniCameraGeometry
    reprojectionError = DistortedOmniReprojectionError
    reprojectionErrorSimple = DistortedOmniReprojectionErrorSimple
    designVariable = DistortedOmniCameraGeometryDesignVariable
    projectionType = aslam_cv.DistortedOmniProjection
    distortionType = aslam_cv.RadialTangentialDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.DistortedOmniFrame

class DistortedOmniRs(CameraModel):
    geometry = aslam_cv.DistortedOmniRsCameraGeometry
    reprojectionError = DistortedOmniRsReprojectionError
    reprojectionErrorSimple = DistortedOmniRsReprojectionErrorSimple
    designVariable = DistortedOmniRsCameraGeometryDesignVariable
    projectionType = aslam_cv.DistortedOmniProjection
    distortionType = aslam_cv.RadialTangentialDistortion
    shutterType = aslam_cv.RollingShutter

class DistortedPinhole(CameraModel):
    geometry = aslam_cv.DistortedPinholeCameraGeometry
    reprojectionError = DistortedPinholeReprojectionError
    reprojectionErrorSimple = DistortedPinholeReprojectionErrorSimple
    designVariable = DistortedPinholeCameraGeometryDesignVariable
    projectionType = aslam_cv.DistortedPinholeProjection
    distortionType = aslam_cv.RadialTangentialDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.DistortedPinholeFrame

class DistortedPinholeRs(CameraModel):
    geometry = aslam_cv.DistortedPinholeRsCameraGeometry
    reprojectionError = DistortedPinholeRsReprojectionError
    reprojectionErrorSimple = DistortedPinholeRsReprojectionErrorSimple
    designVariable = DistortedPinholeRsCameraGeometryDesignVariable
    projectionType = aslam_cv.DistortedPinholeProjection
    distortionType = aslam_cv.RadialTangentialDistortion
    shutterType = aslam_cv.RollingShutter

class EquidistantPinhole(CameraModel):
    geometry = aslam_cv.EquidistantDistortedPinholeCameraGeometry
    reprojectionError = EquidistantDistortedPinholeReprojectionError
    reprojectionErrorSimple = EquidistantDistortedPinholeReprojectionErrorSimple
    designVariable = EquidistantDistortedPinholeCameraGeometryDesignVariable
    projectionType = aslam_cv.EquidistantPinholeProjection
    distortionType = aslam_cv.EquidistantDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.EquidistantDistortedPinholeFrame

class EquidistantPinholeRs(CameraModel):
    geometry = aslam_cv.EquidistantDistortedPinholeRsCameraGeometry
    reprojectionError = EquidistantDistortedPinholeRsReprojectionError
    reprojectionErrorSimple = EquidistantDistortedPinholeRsReprojectionErrorSimple
    designVariable = EquidistantDistortedPinholeRsCameraGeometryDesignVariable
    projectionType = aslam_cv.EquidistantPinholeProjection
    distortionType = aslam_cv.EquidistantDistortion
    shutterType = aslam_cv.RollingShutter

