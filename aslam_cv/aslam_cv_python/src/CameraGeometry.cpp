#include <vector>
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <sm/python/Id.hpp>
#include <boost/python/stl_iterator.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>

namespace detail {
// "Pass by reference" doesn't work with the Eigen type converters.
// So these functions must be wrapped. While we are at it, why 
// not make them nice.
template<typename C>
Eigen::VectorXd e2k(const C * camera, Eigen::Vector3d const & p) {
  Eigen::VectorXd k;
  camera->vsEuclideanToKeypoint(p, k);
  return k;
}

template<typename C>
boost::python::tuple e2kJp(const C * camera, Eigen::Vector3d const & p) {
  Eigen::MatrixXd Jp;
  Eigen::VectorXd k;
  bool isValid = camera->vsEuclideanToKeypoint(p, k, Jp);
  return boost::python::make_tuple(k, Jp, isValid);
}

template<typename C>
Eigen::VectorXd eh2k(const C * camera, Eigen::Vector4d const & p) {
  Eigen::VectorXd k;
  camera->vsHomogeneousToKeypoint(p, k);
  return k;
}

template<typename C>
boost::python::tuple eh2kJp(const C * camera, Eigen::Vector4d const & p) {
  Eigen::MatrixXd Jp;
  Eigen::VectorXd k;
  bool valid = camera->vsHomogeneousToKeypoint(p, k, Jp);
  return boost::python::make_tuple(k, Jp, valid);
}

template<typename C>
Eigen::Vector3d k2e(const C * camera, Eigen::VectorXd const & k) {
  Eigen::Vector3d p;
  camera->vsKeypointToEuclidean(k, p);
  return p;
}

template<typename C>
boost::python::tuple k2eJk(const C * camera, Eigen::VectorXd const & k) {
  Eigen::MatrixXd Jk;
  Eigen::VectorXd p;
  bool valid = camera->vsKeypointToEuclidean(k, p, Jk);
  return boost::python::make_tuple(p, Jk, valid);
}

template<typename C>
Eigen::Vector4d k2eh(const C * camera, Eigen::VectorXd const & k) {
  Eigen::VectorXd ph;
  camera->vsKeypointToHomogeneous(k, ph);
  return ph;
}

template<typename C>
boost::python::tuple k2ehJk(const C * camera, Eigen::VectorXd const & k) {
  Eigen::MatrixXd Jk;
  Eigen::VectorXd p;
  bool valid = camera->vsKeypointToHomogeneous(k, p, Jk);
  return boost::python::make_tuple(p, Jk, valid);
}

template<typename C>
boost::python::tuple estimateTransformation(const C * camera, aslam::cameras::GridCalibrationTargetObservation & obs)
{
  sm::kinematics::Transformation trafo;
  bool success = camera->estimateTransformation(obs, trafo);
  return boost::python::make_tuple(success, trafo);
}

template<typename C>
bool initializeIntrinsics(C* camera, const boost::python::object& py_obslist)
{
  //convert python list to stl vector
  boost::python::stl_input_iterator<aslam::cameras::GridCalibrationTargetObservation> begin(py_obslist), end;
  std::vector<aslam::cameras::GridCalibrationTargetObservation> obslist(begin, end);

  bool success = camera->initializeIntrinsics(obslist);
  return success;
}

}  // namespace detail

template<typename T>
Eigen::MatrixXd getParameters(T * D, bool p, bool d, bool s) {

  Eigen::MatrixXd P;
  D->getParameters(P, p, d, s);
  return P;
}

void exportCameraGeometryBase() {
  sm::python::Id_python_converter<aslam::cameras::CameraId>::register_converter();

  using aslam::cameras::CameraGeometryBase;
  boost::python::class_<aslam::cameras::CameraGeometryBase,
      boost::shared_ptr<aslam::cameras::CameraGeometryBase>, boost::noncopyable>(
      "CameraGeometryBase", boost::python::no_init)
      .def("keypointDimension", &CameraGeometryBase::keypointDimension, "Get the dimension of the keypoint type")
      .def("isProjectionInvertible", &CameraGeometryBase::isProjectionInvertible, "Is the sensor model invertible? Ususally this is only true for a range/bearing sensor.")
      .def("temporalOffset", &CameraGeometryBase::vsTemporalOffset, "Given a keypoint, what is the offset from the start of integration for this image?\nDuration = temporalOffset(keypoint)")
      .def("createRandomKeypoint", &CameraGeometryBase::createRandomKeypoint, "Create a valid, random keypoint. This is useful for unit testing and experiments.")
      .def("createRandomVisiblePoint",&CameraGeometryBase::createRandomVisiblePoint, "Create a valid point in space visible by the camera.\np = createRandomVisiblePoint(depth).")
      .def("euclideanToKeypoint", &detail::e2k<CameraGeometryBase>, "Map a 3x1 Euclidean point to a keypoint.\nk = euclideanToKeypoint(p)")
      .def("euclideanToKeypointJp", &detail::e2kJp<CameraGeometryBase>, "Map a 3x1 Euclidean point to a keypoint and get the Jacobian of the mapping with respect to small changes in the point.\n(k, Jp) = euclideanToKeypoint(p)")
      .def("homogeneousToKeypoint", &detail::eh2k<CameraGeometryBase>, "Map a 4x1 homogeneous Euclidean point to a keypoint.\nk = euclideanToKeypoint(p)")
      .def("homogeneousToKeypointJp", &detail::eh2kJp<CameraGeometryBase>, "Map a 4x1 homogeneous Euclidean point to a keypoint and get the Jacobian of the mapping with respect to small changes in the point.\n(k, Jp) = homogeneousToKeypointJp(p)")
      .def("keypointToHomogeneous", &detail::k2eh<CameraGeometryBase>,"Map a keypoint to a 4x1 homogeneous Euclidean point.\np = keypointToHomogeneous(k)")
      .def("keypointToHomogeneousJk", &detail::k2ehJk<CameraGeometryBase>, "Map a keypoint to a 4x1 homogeneous Euclidean point and get the Jacobian of the mapping with respect to small changes in the keypoint.\n(p, Jk) = keypointToHomogeneousJk(k)")
      .def("keypointToEuclidean", &detail::k2e<CameraGeometryBase>, "Map a keypoint to a 3x1 Euclidean point.\np = keypointToEuclidean(k)")
      .def("keypointToEuclideanJk", &detail::k2eJk<CameraGeometryBase>, "Map a keypoint to a 3x1 Euclidean point and get the Jacobian of the mapping with respect to small changes in the keypoint.\n(p, Jk) = keypointToEuclideanJk(k)")
      .def("isValid", &CameraGeometryBase::vsIsValid)
      .def("isEuclideanVisible", &CameraGeometryBase::vsIsEuclideanVisible)
      .def("isHomogeneousVisible", &CameraGeometryBase::vsIsHomogeneousVisible)
      .def("minimalDimensionsProjection", &CameraGeometryBase::minimalDimensionsProjection)
      .def("minimalDimensionsDistortion", &CameraGeometryBase::minimalDimensionsDistortion)
      .def("minimalDimensionsShutter", &CameraGeometryBase::minimalDimensionsShutter)
      .def("getParameters", &getParameters<CameraGeometryBase>)
      .def("setParameters", &CameraGeometryBase::setParameters)
      .def("estimateTransformation", &detail::estimateTransformation<CameraGeometryBase>, "estimate the transformation of the camera with respect to the calibration target, returns tuple (bool, sm.Transformation)")
      .def("initializeIntrinsics", &detail::initializeIntrinsics<CameraGeometryBase>, "intialize intrinsics on a list of observations")
      .def_pickle( sm::python::pickle_suite<CameraGeometryBase>())
      ;
}
