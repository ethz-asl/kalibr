// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/ExportReprojectionError.hpp>
#include <aslam/ExportCameraDesignVariable.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/RollingShutter.hpp>

void exportGridCalibration();
// void exportCameraGeometryDvc();
//void exportNCameraSystemDvc();

BOOST_PYTHON_MODULE(libaslam_cv_backend_python)
{
  exportGridCalibration();
  // exportCameraGeometryDvc();
  //exportNCameraSystemDvc();

  using namespace aslam::cameras;
  aslam::python::exportReprojectionErrors<PinholeCameraGeometry>("Pinhole");
  aslam::python::exportReprojectionErrors<PinholeRsCameraGeometry>("PinholeRs");
  aslam::python::exportCovarianceReprojectionError<PinholeRsCameraGeometry>("PinholeRs");

  aslam::python::exportReprojectionErrors<DistortedPinholeCameraGeometry>("DistortedPinhole");
  aslam::python::exportReprojectionErrors<DistortedPinholeRsCameraGeometry>("DistortedPinholeRs");
  aslam::python::exportCovarianceReprojectionError<DistortedPinholeRsCameraGeometry>("DistortedPinholeRs");

  aslam::python::exportReprojectionErrors<EquidistantDistortedPinholeCameraGeometry>("EquidistantDistortedPinhole");
  aslam::python::exportReprojectionErrors<EquidistantDistortedPinholeRsCameraGeometry>("EquidistantDistortedPinholeRs");
  aslam::python::exportCovarianceReprojectionError<EquidistantDistortedPinholeRsCameraGeometry>("EquidistantDistortedPinholeRs");

  aslam::python::exportReprojectionErrors<FovDistortedPinholeCameraGeometry>("FovDistortedPinhole");
  aslam::python::exportReprojectionErrors<FovDistortedPinholeRsCameraGeometry>("FovDistortedPinholeRs");
  aslam::python::exportCovarianceReprojectionError<FovDistortedPinholeRsCameraGeometry>("FovDistortedPinholeRs");

  aslam::python::exportReprojectionErrors<ExtendedUnifiedCameraGeometry>("ExtendedUnified");
  aslam::python::exportReprojectionErrors<DoubleSphereCameraGeometry>("DoubleSphere");

  aslam::python::exportReprojectionErrors<OmniCameraGeometry>("Omni");
  aslam::python::exportReprojectionErrors<OmniRsCameraGeometry>("OmniRs");
  aslam::python::exportCovarianceReprojectionError<OmniRsCameraGeometry>("OmniRs");

  aslam::python::exportReprojectionErrors<DistortedOmniCameraGeometry>("DistortedOmni");
  aslam::python::exportReprojectionErrors<DistortedOmniRsCameraGeometry>("DistortedOmniRs");
  aslam::python::exportCovarianceReprojectionError<DistortedOmniRsCameraGeometry>("DistortedOmniRs");

  aslam::python::exportReprojectionErrors<EquidistantDistortedOmniCameraGeometry>("EquidistantDistortedOmni");
  aslam::python::exportReprojectionErrors<EquidistantDistortedOmniRsCameraGeometry>("EquidistantDistortedOmniRs");
  aslam::python::exportCovarianceReprojectionError<EquidistantDistortedOmniRsCameraGeometry>("EquidistantDistortedOmniRs");

  aslam::python::exportReprojectionErrors<FovDistortedOmniCameraGeometry>("FovDistortedOmni");
  aslam::python::exportReprojectionErrors<FovDistortedOmniRsCameraGeometry>("FovDistortedOmniRs");
  aslam::python::exportCovarianceReprojectionError<FovDistortedOmniRsCameraGeometry>("FovDistortedOmniRs");


  // Export the camera design variables:
  using namespace aslam::python;

  exportCameraDesignVariables<PinholeCameraGeometry>("PinholeCameraGeometry");
  exportCameraDesignVariables<DistortedPinholeCameraGeometry>("DistortedPinholeCameraGeometry");
  exportCameraDesignVariables<EquidistantDistortedPinholeCameraGeometry>("EquidistantDistortedPinholeCameraGeometry");
  exportCameraDesignVariables<FovDistortedPinholeCameraGeometry>("FovDistortedPinholeCameraGeometry");

  exportCameraDesignVariables<PinholeRsCameraGeometry>("PinholeRsCameraGeometry");
  exportCameraDesignVariables<DistortedPinholeRsCameraGeometry>("DistortedPinholeRsCameraGeometry");
  exportCameraDesignVariables<EquidistantDistortedPinholeRsCameraGeometry>("EquidistantDistortedPinholeRsCameraGeometry");
  exportCameraDesignVariables<FovDistortedPinholeRsCameraGeometry>("FovDistortedPinholeRsCameraGeometry");

  exportCameraDesignVariables<OmniRsCameraGeometry>("OmniRsCameraGeometry");
  exportCameraDesignVariables<DistortedOmniRsCameraGeometry>("DistortedOmniRsCameraGeometry");
  exportCameraDesignVariables<EquidistantDistortedOmniRsCameraGeometry>("EquidistantDistortedOmniRsCameraGeometry");
  exportCameraDesignVariables<FovDistortedOmniRsCameraGeometry>("FovDistortedOmniRsCameraGeometry");

  exportCameraDesignVariables<ExtendedUnifiedCameraGeometry>("ExtendedUnifiedCameraGeometry");
  exportCameraDesignVariables<DoubleSphereCameraGeometry>("DoubleSphereCameraGeometry");

  exportCameraDesignVariables<OmniCameraGeometry>("OmniCameraGeometry");
  exportCameraDesignVariables<DistortedOmniCameraGeometry>("DistortedOmniCameraGeometry");
  exportCameraDesignVariables<EquidistantDistortedOmniCameraGeometry>("EquidistantDistortedOmniCameraGeometry");
  exportCameraDesignVariables<FovDistortedOmniCameraGeometry>("FovDistortedOmniCameraGeometry");

  exportCameraDesignVariables<MaskedPinholeCameraGeometry>("MaskedPinholeCameraGeometry");
  exportCameraDesignVariables<MaskedDistortedPinholeCameraGeometry>("MaskedDistortedPinholeCameraGeometry");
  exportCameraDesignVariables<MaskedEquidistantDistortedPinholeCameraGeometry>("MaskedEquidistantDistortedPinholeCameraGeometry");
  exportCameraDesignVariables<MaskedFovDistortedPinholeCameraGeometry>("MaskedFovDistortedPinholeCameraGeometry");

  exportCameraDesignVariables<MaskedPinholeRsCameraGeometry>("MaskedPinholeRsCameraGeometry");
  exportCameraDesignVariables<MaskedDistortedPinholeRsCameraGeometry>("MaskedDistortedPinholeRsCameraGeometry");
  exportCameraDesignVariables<MaskedEquidistantDistortedPinholeRsCameraGeometry>("MaskedEquidistantDistortedPinholeRsCameraGeometry");
  exportCameraDesignVariables<MaskedFovDistortedPinholeRsCameraGeometry>("MaskedFovDistortedPinholeRsCameraGeometry");

  exportCameraDesignVariables<MaskedOmniRsCameraGeometry>("MaskedOmniRsCameraGeometry");
  exportCameraDesignVariables<MaskedDistortedOmniRsCameraGeometry>("MaskedDistortedOmniRsCameraGeometry");
  exportCameraDesignVariables<MaskedEquidistantDistortedOmniRsCameraGeometry>("MaskedEquidistantDistortedOmniRsCameraGeometry");
  exportCameraDesignVariables<MaskedFovDistortedOmniRsCameraGeometry>("MaskedFovDistortedOmniRsCameraGeometry");

  exportCameraDesignVariables<MaskedOmniCameraGeometry>("MaskedOmniCameraGeometry");
  exportCameraDesignVariables<MaskedDistortedOmniCameraGeometry>("MaskedDistortedOmniCameraGeometry");
  exportCameraDesignVariables<MaskedEquidistantDistortedOmniCameraGeometry>("MaskedEquidistantDistortedOmniCameraGeometry");
  exportCameraDesignVariables<MaskedFovDistortedOmniCameraGeometry>("MaskedFovDistortedOmniCameraGeometry");

  exportCameraDesignVariables<DepthCameraGeometry>("DepthCameraGeometry");
  exportCameraDesignVariables<DistortedDepthCameraGeometry>("DistortedDepthCameraGeometry");
  exportCameraDesignVariables<EquidistantDistortedDepthCameraGeometry>("EquidistantDistortedDepthCameraGeometry");
  exportCameraDesignVariables<FovDistortedDepthCameraGeometry>("FovDistortedDepthCameraGeometry");

  // Export Projection and Distortion Model Design Variables:
  exportGenericProjectionDesignVariable<NoDistortion>("NoDistortion");
  exportGenericProjectionDesignVariable<RadialTangentialDistortion>("RadialTangentialDistortion");
  exportGenericProjectionDesignVariable<EquidistantDistortion>("EquidistantDistortion");
  exportGenericProjectionDesignVariable<FovDistortion>("FovDistortion");

  exportGenericProjectionDesignVariable< PinholeProjection<NoDistortion> >("PinholeProjection");
  exportGenericProjectionDesignVariable< PinholeProjection<RadialTangentialDistortion> >("DistortedPinholeProjection");
  exportGenericProjectionDesignVariable< PinholeProjection<EquidistantDistortion> >("EquidistantDistortedPinholeProjection");
  exportGenericProjectionDesignVariable< PinholeProjection<FovDistortion> >("FovDistortedPinholeProjection");

  exportGenericProjectionDesignVariable< ExtendedUnifiedProjection<NoDistortion> >("ExtendedUnifiedProjection");
  exportGenericProjectionDesignVariable< DoubleSphereProjection<NoDistortion> >("DoubleSphereProjection");

  exportGenericProjectionDesignVariable< OmniProjection<NoDistortion> >("OmniProjection");
  exportGenericProjectionDesignVariable< OmniProjection<RadialTangentialDistortion> >("DistortedOmniProjection");
  exportGenericProjectionDesignVariable< OmniProjection<EquidistantDistortion> >("EquidistantDistortedOmniProjection");
  exportGenericProjectionDesignVariable< OmniProjection<FovDistortion> >("FovDistortedOmniProjection");

  // Export Shutter Design Variables:
  exportShutterDesignVariable< GlobalShutter >("GlobalShutter");
  exportShutterDesignVariable< RollingShutter >("RollingShutter");

}
