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

class Omni(CameraModel):
    geometry = aslam_cv.OmniCameraGeometry
    reprojectionError = OmniReprojectionError
    reprojectionErrorSimple = OmniReprojectionErrorSimple
    designVariable = OmniCameraGeometryDesignVariable
    projectionType = aslam_cv.OmniProjection
    distortionType = aslam_cv.NoDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.OmniFrame

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
    reprojectionErrorAdaptiveCovariance = DistortedOmniRsReprojectionErrorAdaptiveCovariance
    designVariable = DistortedOmniRsCameraGeometryDesignVariable
    projectionType = aslam_cv.DistortedOmniProjection
    distortionType = aslam_cv.RadialTangentialDistortion
    shutterType = aslam_cv.RollingShutter
    frameType = aslam_cv.DistortedOmniRsFrame

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
    reprojectionErrorAdaptiveCovariance = DistortedPinholeRsReprojectionErrorAdaptiveCovariance
    designVariable = DistortedPinholeRsCameraGeometryDesignVariable
    projectionType = aslam_cv.DistortedPinholeProjection
    distortionType = aslam_cv.RadialTangentialDistortion
    shutterType = aslam_cv.RollingShutter
    frameType = aslam_cv.DistortedPinholeRsFrame

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
    reprojectionErrorAdaptiveCovariance = EquidistantDistortedPinholeRsReprojectionErrorAdaptiveCovariance
    designVariable = EquidistantDistortedPinholeRsCameraGeometryDesignVariable
    projectionType = aslam_cv.EquidistantPinholeProjection
    distortionType = aslam_cv.EquidistantDistortion
    shutterType = aslam_cv.RollingShutter
    frameType = aslam_cv.EquidistantPinholeRsFrame

class FovPinhole(CameraModel):
    geometry = aslam_cv.FovDistortedPinholeCameraGeometry
    reprojectionError = FovDistortedPinholeReprojectionError
    reprojectionErrorSimple = FovDistortedPinholeReprojectionErrorSimple
    designVariable = FovDistortedPinholeCameraGeometryDesignVariable
    projectionType = aslam_cv.FovPinholeProjection
    distortionType = aslam_cv.FovDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.FovDistortedPinholeFrame

class ExtendedUnified(CameraModel):
    geometry = aslam_cv.ExtendedUnifiedCameraGeometry
    reprojectionError = ExtendedUnifiedReprojectionError
    reprojectionErrorSimple = ExtendedUnifiedReprojectionErrorSimple
    designVariable = ExtendedUnifiedCameraGeometryDesignVariable
    projectionType = aslam_cv.ExtendedUnifiedProjection
    distortionType = aslam_cv.NoDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.ExtendedUnifiedFrame

class DoubleSphere(CameraModel):
    geometry = aslam_cv.DoubleSphereCameraGeometry
    reprojectionError = DoubleSphereReprojectionError
    reprojectionErrorSimple = DoubleSphereReprojectionErrorSimple
    designVariable = DoubleSphereCameraGeometryDesignVariable
    projectionType = aslam_cv.DoubleSphereProjection
    distortionType = aslam_cv.NoDistortion
    shutterType = aslam_cv.GlobalShutter
    frameType = aslam_cv.DoubleSphereFrame

