#include <aslam/UndistorterBase.hpp>
#include <aslam/OmniUndistorter.hpp>
#include <aslam/PinholeUndistorter.hpp>
#include <aslam/NullUndistorter.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/cameras.hpp>

namespace aslam {

UndistorterBase::UndistorterBase() {
}
UndistorterBase::~UndistorterBase() {
}

boost::shared_ptr<UndistorterBase> UndistorterBase::createUndistorter(
    const sm::PropertyTree & undistorterConfig,
    const sm::PropertyTree & cameraConfig) {

  using namespace aslam::cameras;

  std::string type = undistorterConfig.getString("type");

  boost::shared_ptr<UndistorterBase> rval;
  if (type == "NullPinholeCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter,
                NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullDistortedPinholeCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
                GlobalShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullEquidistantDistortedPinholeCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<EquidistantDistortion>,
                GlobalShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullOmniCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter, NoMask> >(
            undistorterConfig, cameraConfig));
  } else if (type == "NullDistortedOmniCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<RadialTangentialDistortion>,
                GlobalShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullEquidistantDistortedOmniCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
                NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullPinholeRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter,
                NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullDistortedPinholeRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
                RollingShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullEquidistantDistortedPinholeRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<EquidistantDistortion>,
                RollingShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullOmniRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<NoDistortion>, RollingShutter, NoMask> >(
            undistorterConfig, cameraConfig));
  } else if (type == "NullDistortedOmniRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<RadialTangentialDistortion>,
                RollingShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullEquidistantDistortedOmniRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<EquidistantDistortion>,
                RollingShutter, NoMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedPinholeCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter,
                ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedDistortedPinholeCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
                GlobalShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedEquidistantDistortedPinholeCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<EquidistantDistortion>,
                GlobalShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedOmniCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter,
                ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedDistortedOmniCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<RadialTangentialDistortion>,
                GlobalShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedEquidistantDistortedOmniCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
                ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedPinholeRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter,
                ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedDistortedPinholeRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
                RollingShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedEquidistantDistortedPinholeRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<PinholeProjection<EquidistantDistortion>,
                RollingShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedOmniRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<NoDistortion>, RollingShutter,
                ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedDistortedOmniRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<RadialTangentialDistortion>,
                RollingShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "NullMaskedEquidistantDistortedOmniRsCameraGeometry") {
    rval.reset(
        new NullUndistorter<
            CameraGeometry<OmniProjection<EquidistantDistortion>,
                RollingShutter, ImageMask> >(undistorterConfig, cameraConfig));
  } else if (type == "MaskedPinholeUndistorter") {
    rval.reset(
        new PinholeUndistorter<RadialTangentialDistortion, ImageMask>(undistorterConfig, cameraConfig));
  } else if (type == "PinholeUndistorter") {
    rval.reset(new PinholeUndistorter<RadialTangentialDistortion, NoMask>(undistorterConfig, cameraConfig));
  } else if (type == "MaskedEquidistantPinholeUndistorter") {
    rval.reset(new PinholeUndistorter<EquidistantDistortion, ImageMask>(undistorterConfig, cameraConfig));
  } else if (type == "EquidistantPinholeUndistorter") {
    rval.reset(new PinholeUndistorter<EquidistantDistortion, NoMask>(undistorterConfig, cameraConfig));
  } else if (type == "MaskedOmniUndistorter") {
    rval.reset(new OmniUndistorter<ImageMask>(undistorterConfig, cameraConfig));
  } else if (type == "OmniUndistorter") {
    rval.reset(new OmniUndistorter<NoMask>(undistorterConfig, cameraConfig));
  } else {
    SM_THROW(
        std::runtime_error,
        "Unknown undistorter type \"" << type
            << "\". Try reading the file \"aslam_cv/aslam_imgproc/src/UndistorterBase.cpp\" for a list of valid undistorters");
  }

  return rval;
}
}  // namespace aslam
