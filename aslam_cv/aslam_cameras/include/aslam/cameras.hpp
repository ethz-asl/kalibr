#ifndef ASLAM_CAMERAS_HPP
#define ASLAM_CAMERAS_HPP

// The camera geometery mother class
#include <aslam/cameras/CameraGeometry.hpp>

// Projection models
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/DepthProjection.hpp>

// Distortion models
#include <aslam/cameras/NoDistortion.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>

// Masks
#include <aslam/cameras/NoMask.hpp>
#include <aslam/cameras/ImageMask.hpp>

// Shutter models
#include <aslam/cameras/RollingShutter.hpp>
#include <aslam/cameras/GlobalShutter.hpp>

namespace aslam {
namespace cameras {

typedef CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter, NoMask> PinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    GlobalShutter, NoMask> DistortedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, GlobalShutter,
    NoMask> EquidistantDistortedPinholeCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter, NoMask> OmniCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    GlobalShutter, NoMask> DistortedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
    NoMask> EquidistantDistortedOmniCameraGeometry;

typedef CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter, NoMask> PinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    RollingShutter, NoMask> DistortedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, RollingShutter,
    NoMask> EquidistantDistortedPinholeRsCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, RollingShutter, NoMask> OmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    RollingShutter, NoMask> DistortedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, RollingShutter,
    NoMask> EquidistantDistortedOmniRsCameraGeometry;

typedef CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter, ImageMask> MaskedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    GlobalShutter, ImageMask> MaskedDistortedPinholeCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, GlobalShutter,
    ImageMask> MaskedEquidistantDistortedPinholeCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter, ImageMask> MaskedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    GlobalShutter, ImageMask> MaskedDistortedOmniCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
    ImageMask> MaskedEquidistantDistortedOmniCameraGeometry;

typedef CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter,
    ImageMask> MaskedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
    RollingShutter, ImageMask> MaskedDistortedPinholeRsCameraGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>, RollingShutter,
    ImageMask> MaskedEquidistantDistortedPinholeRsCameraGeometry;

typedef CameraGeometry<OmniProjection<NoDistortion>, RollingShutter, ImageMask> MaskedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<RadialTangentialDistortion>,
    RollingShutter, ImageMask> MaskedDistortedOmniRsCameraGeometry;
typedef CameraGeometry<OmniProjection<EquidistantDistortion>, RollingShutter,
    ImageMask> MaskedEquidistantDistortedOmniRsCameraGeometry;

typedef CameraGeometry<DepthProjection<NoDistortion>, GlobalShutter, NoMask> DepthCameraGeometry;
typedef CameraGeometry<DepthProjection<RadialTangentialDistortion>,
    GlobalShutter, NoMask> DistortedDepthCameraGeometry;
typedef CameraGeometry<DepthProjection<EquidistantDistortion>, GlobalShutter,
    NoMask> EquidistantDistortedDepthCameraGeometry;

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_CAMERAS_HPP */
