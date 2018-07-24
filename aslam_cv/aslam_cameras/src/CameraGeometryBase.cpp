#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras.hpp>
namespace aslam {
namespace cameras {

CameraGeometryBase::CameraGeometryBase() {
}
;
CameraGeometryBase::~CameraGeometryBase() {
}
;

boost::shared_ptr<CameraGeometryBase> CameraGeometryBase::create(
    const sm::PropertyTree & config) {

  std::string type = config.getString("type");

  boost::shared_ptr<CameraGeometryBase> rval;
  if (type == "Pinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter,
            NoMask>(config));
  } else if (type == "DistortedPinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
            GlobalShutter, NoMask>(config));
  } else if (type == "EquidistantDistortedPinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<EquidistantDistortion>,
            GlobalShutter, NoMask>(config));
  } else if (type == "FovDistortedPinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<FovDistortion>,
            GlobalShutter, NoMask>(config));
  } else if (type == "Omni") {
    rval.reset(
        new CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter, NoMask>(
            config));
  } else if (type == "DistortedOmni") {
    rval.reset(
        new CameraGeometry<OmniProjection<RadialTangentialDistortion>,
            GlobalShutter, NoMask>(config));
  } else if (type == "EquidistantDistortedOmni") {
    rval.reset(
        new CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
            NoMask>(config));
  } else if (type == "ExtendedUnified") {
    rval.reset(
        new CameraGeometry<ExtendedUnifiedProjection<NoDistortion>, GlobalShutter, NoMask>(
            config));
  } else if (type == "DoubleSphere") {
    rval.reset(
        new CameraGeometry<DoubleSphereProjection<NoDistortion>, GlobalShutter, NoMask>(
            config));
  } else if (type == "PinholeRs") {
    rval.reset(
        new CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter,
            NoMask>(config));
  } else if (type == "DistortedPinholeRs") {
    rval.reset(
        new CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
            RollingShutter, NoMask>(config));
  } else if (type == "EquidistantDistortedPinholeRs") {
    rval.reset(
        new CameraGeometry<PinholeProjection<EquidistantDistortion>,
            RollingShutter, NoMask>(config));
  } else if (type == "OmniRs") {
    rval.reset(
        new CameraGeometry<OmniProjection<NoDistortion>, RollingShutter, NoMask>(
            config));
  } else if (type == "DistortedOmniRs") {
    rval.reset(
        new CameraGeometry<OmniProjection<RadialTangentialDistortion>,
            RollingShutter, NoMask>(config));
  } else if (type == "EquidistantDistortedOmniRs") {
    rval.reset(
        new CameraGeometry<OmniProjection<EquidistantDistortion>,
            RollingShutter, NoMask>(config));
  } else if (type == "MaskedPinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<NoDistortion>, GlobalShutter,
            ImageMask>(config));
  } else if (type == "MaskedDistortedPinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
            GlobalShutter, ImageMask>(config));
  } else if (type == "MaskedEquidistantDistortedPinhole") {
    rval.reset(
        new CameraGeometry<PinholeProjection<EquidistantDistortion>,
            GlobalShutter, ImageMask>(config));
  } else if (type == "MaskedOmni") {
    rval.reset(
        new CameraGeometry<OmniProjection<NoDistortion>, GlobalShutter,
            ImageMask>(config));
  } else if (type == "MaskedDistortedOmni") {
    rval.reset(
        new CameraGeometry<OmniProjection<RadialTangentialDistortion>,
            GlobalShutter, ImageMask>(config));
  } else if (type == "MaskedEquidistantDistortedOmni") {
    rval.reset(
        new CameraGeometry<OmniProjection<EquidistantDistortion>, GlobalShutter,
            ImageMask>(config));
  } else if (type == "MaskedPinholeRs") {
    rval.reset(
        new CameraGeometry<PinholeProjection<NoDistortion>, RollingShutter,
            ImageMask>(config));
  } else if (type == "MaskedDistortedPinholeRs") {
    rval.reset(
        new CameraGeometry<PinholeProjection<RadialTangentialDistortion>,
            RollingShutter, ImageMask>(config));
  } else if (type == "MaskedEquidistantDistortedPinholeRs") {
    rval.reset(
        new CameraGeometry<PinholeProjection<EquidistantDistortion>,
            RollingShutter, ImageMask>(config));
  } else if (type == "MaskedOmniRs") {
    rval.reset(
        new CameraGeometry<OmniProjection<NoDistortion>, RollingShutter,
            ImageMask>(config));
  } else if (type == "MaskedDistortedOmniRs") {
    rval.reset(
        new CameraGeometry<OmniProjection<RadialTangentialDistortion>,
            RollingShutter, ImageMask>(config));
  } else if (type == "MaskedEquidistantDistortedOmniRs") {
    rval.reset(
        new CameraGeometry<OmniProjection<EquidistantDistortion>,
            RollingShutter, ImageMask>(config));
  } else {
    SM_THROW(
        std::runtime_error,
        "Unknown camera type \"" << type
            << "\". Try reading the file \"aslam_cv/aslam_cameras/src/CameraGeometryBase.cpp\" for a list of valid camera types");
  }
  return rval;
}

bool CameraGeometryBase::hasMask() const
{
	SM_THROW(std::runtime_error, "not implemented!");
	return false;
}


}  // namespace cameras
}  // namespace aslam

