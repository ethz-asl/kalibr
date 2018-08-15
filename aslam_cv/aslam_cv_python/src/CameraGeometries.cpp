#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/cameras.hpp>

//#include <aslam/backend/CameraDesignVariable.hpp>
//#include <aslam/python/ExportBackendExpressions.hpp>
#include <sm/python/Id.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <aslam/cameras/Triangulation.hpp>

#include <aslam/LinkCvSerialization.hpp>

using namespace aslam::cameras;
using namespace boost::python;

void exportCameraGeometryBase();
void exportCameraProjections();
void exportCameraShutters();

boost::python::tuple triangulateWrap(const Eigen::Vector3d & point0,
                                     const Eigen::Vector3d & ray0,
                                     const Eigen::Vector3d & point1,
                                     const Eigen::Vector3d & ray1) {
  Eigen::Vector3d outTriangulatedPoint;
  double outGap;
  double outS0;
  double outS1;
  triangulate(point0, ray0, point1, ray1, outTriangulatedPoint, outGap, outS0,
              outS1);

  return boost::python::make_tuple(outTriangulatedPoint, outGap, outS0, outS1);
}

// template<typename C>
// void exportCameraDesignVariables(std::string name)
// {

// 	using namespace aslam::backend;
// 	using namespace aslam::python;

// 	boost::python::class_<CameraDesignVariable<C>, boost::shared_ptr<CameraDesignVariable<C> > >((name + "DesignVariable").c_str() , boost::python::init< boost::shared_ptr<C> >())
// 			.def("euclideanToKeypoint", &CameraDesignVariable<C>::euclideanToKeypoint)
// 			.def("homogeneousToKeypoint", &CameraDesignVariable<C>::homogeneousToKeypoint)
// 			.def("setActive", &CameraDesignVariable<C>::setActive )
// 			.def("getDesignVariables", &getDesignVariables<CameraDesignVariable<C> >)
// 			.def("projectionDesignVariable", &CameraDesignVariable<C>::projectionDesignVariable)
// 			.def("distortionDesignVariable", &CameraDesignVariable<C>::distortionDesignVariable)
// 			.def("shutterDesignVariable", &CameraDesignVariable<C>::shutterDesignVariable)
// 			.def("camera", &CameraDesignVariable<C>::camera)
// 			;

// }

template<typename C>
void exportCameraGeometry(std::string name) {
  typedef typename C::shutter_t shutter_t;
  typedef typename C::projection_t projection_t;
  typedef typename C::mask_t mask_t;

  shutter_t & (C::*shutter)() = &C::shutter;
  projection_t & (C::*projection)() = &C::projection;
  mask_t & (C::*mask)() = &C::mask;

  boost::python::class_<C, boost::shared_ptr<C>,
      boost::python::bases<CameraGeometryBase> >(name.c_str(), boost::python::init<>())
      .def(init<typename C::projection_t>())
      .def(init<typename C::projection_t, typename C::shutter_t>())
      .def(init<typename C::projection_t, typename C::shutter_t, typename C::mask_t>())
      .def("shutter", shutter, return_internal_reference<>())
      .def("projection", projection, return_internal_reference<>())
      .def("mask", mask, return_internal_reference<>())
      .def("getTestGeometry", C::getTestGeometry)
      .staticmethod("getTestGeometry")
      .def_pickle(sm::python::pickle_suite<C>());

  //exportCameraDesignVariables<C>(name);
}

template<typename T>
bool isValid(const T * mask, const Eigen::VectorXd & v) {
  return mask->isValid(v);
}

void exportCameraGeometries() {
  aslam::linkCvSerialization();

  exportCameraGeometryBase();
  exportCameraProjections();
  exportCameraShutters();

  //exportPinholeCameraGeometry();
  //exportPinholeRSCameraGeometry();
  //exportOmniCameraGeometry();
  exportCameraGeometry<PinholeCameraGeometry>("PinholeCameraGeometry");
  exportCameraGeometry<DistortedPinholeCameraGeometry>(
      "DistortedPinholeCameraGeometry");
  exportCameraGeometry<EquidistantDistortedPinholeCameraGeometry>(
      "EquidistantDistortedPinholeCameraGeometry");
  exportCameraGeometry<FovDistortedPinholeCameraGeometry>(
      "FovDistortedPinholeCameraGeometry");

  exportCameraGeometry<PinholeRsCameraGeometry>("PinholeRsCameraGeometry");
  exportCameraGeometry<DistortedPinholeRsCameraGeometry>(
      "DistortedPinholeRsCameraGeometry");
  exportCameraGeometry<EquidistantDistortedPinholeRsCameraGeometry>(
      "EquidistantDistortedPinholeRsCameraGeometry");
  exportCameraGeometry<FovDistortedPinholeRsCameraGeometry>(
        "FovDistortedPinholeRsCameraGeometry");

  exportCameraGeometry<OmniRsCameraGeometry>("OmniRsCameraGeometry");
  exportCameraGeometry<DistortedOmniRsCameraGeometry>(
      "DistortedOmniRsCameraGeometry");
  exportCameraGeometry<EquidistantDistortedOmniRsCameraGeometry>(
      "EquidistantDistortedOmniRsCameraGeometry");
  exportCameraGeometry<FovDistortedOmniRsCameraGeometry>(
      "FovDistortedOmniRsCameraGeometry");

  exportCameraGeometry<ExtendedUnifiedCameraGeometry>("ExtendedUnifiedCameraGeometry");

  exportCameraGeometry<DoubleSphereCameraGeometry>("DoubleSphereCameraGeometry");

  exportCameraGeometry<OmniCameraGeometry>("OmniCameraGeometry");
  exportCameraGeometry<DistortedOmniCameraGeometry>(
      "DistortedOmniCameraGeometry");
  exportCameraGeometry<EquidistantDistortedOmniCameraGeometry>(
      "EquidistantDistortedOmniCameraGeometry");
  exportCameraGeometry<FovDistortedOmniCameraGeometry>(
      "FovDistortedOmniCameraGeometry");

  exportCameraGeometry<MaskedPinholeCameraGeometry>(
      "MaskedPinholeCameraGeometry");
  exportCameraGeometry<MaskedDistortedPinholeCameraGeometry>(
      "MaskedDistortedPinholeCameraGeometry");
  exportCameraGeometry<MaskedEquidistantDistortedPinholeCameraGeometry>(
      "MaskedEquidistantDistortedPinholeCameraGeometry");
  exportCameraGeometry<MaskedFovDistortedPinholeCameraGeometry>(
      "MaskedFovDistortedPinholeCameraGeometry");

  exportCameraGeometry<MaskedPinholeRsCameraGeometry>(
      "MaskedPinholeRsCameraGeometry");
  exportCameraGeometry<MaskedDistortedPinholeRsCameraGeometry>(
      "MaskedDistortedPinholeRsCameraGeometry");
  exportCameraGeometry<MaskedEquidistantDistortedPinholeRsCameraGeometry>(
      "MaskedEquidistantDistortedPinholeRsCameraGeometry");
  exportCameraGeometry<MaskedFovDistortedPinholeRsCameraGeometry>(
      "MaskedFovDistortedPinholeRsCameraGeometry");

  exportCameraGeometry<MaskedOmniRsCameraGeometry>(
      "MaskedOmniRsCameraGeometry");
  exportCameraGeometry<MaskedDistortedOmniRsCameraGeometry>(
      "MaskedDistortedOmniRsCameraGeometry");
  exportCameraGeometry<MaskedEquidistantDistortedOmniRsCameraGeometry>(
      "MaskedEquidistantDistortedOmniRsCameraGeometry");
  exportCameraGeometry<MaskedFovDistortedOmniRsCameraGeometry>(
      "MaskedFovDistortedOmniRsCameraGeometry");

  exportCameraGeometry<MaskedOmniCameraGeometry>("MaskedOmniCameraGeometry");
  exportCameraGeometry<MaskedDistortedOmniCameraGeometry>(
      "MaskedDistortedOmniCameraGeometry");
  exportCameraGeometry<MaskedEquidistantDistortedOmniCameraGeometry>(
      "MaskedEquidistantDistortedOmniCameraGeometry");
  exportCameraGeometry<MaskedFovDistortedOmniCameraGeometry>(
      "MaskedFovDistortedOmniCameraGeometry");

  exportCameraGeometry<DepthCameraGeometry>("DepthCameraGeometry");
  exportCameraGeometry<DistortedDepthCameraGeometry>(
      "DistortedDepthCameraGeometry");
  exportCameraGeometry<EquidistantDistortedDepthCameraGeometry>(
      "EquidistantDistortedDepthCameraGeometry");
  exportCameraGeometry<FovDistortedDepthCameraGeometry>(
      "FovDistortedDepthCameraGeometry");

  class_<ImageMask, boost::shared_ptr<ImageMask> >("ImageMask", init<>())
      .def("setMask", &ImageMask::setMaskFromMatrix)
      .def("getMask", &ImageMask::getMaskAsMatrix)
      .def("isValid", &ImageMask::isValid<Eigen::VectorXd>);

  class_<NoMask, boost::shared_ptr<NoMask> >("NoMask", init<>())
      .def("isValid", &NoMask::isValid<Eigen::VectorXd>);

  def("triangulate", &triangulateWrap,
      "(outTriangulatedPoint, outGap, outS0, outS1 ) = triangulate( point0, ray0, point1, ray1)");
}
