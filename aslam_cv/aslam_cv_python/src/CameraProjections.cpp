#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/ExtendedUnifiedProjection.hpp>
#include <aslam/cameras/DoubleSphereProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras/FovDistortion.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>
#include <boost/serialization/nvp.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <sm/python/unique_register_ptr_to_python.hpp>
//#include <aslam/python/ExportDesignVariableAdapter.hpp>
//#include <aslam/backend/DesignVariableAdapter.hpp>
//#include <aslam/python/ExportAPrioriInformationError.hpp>

using namespace boost::python;
using namespace aslam::cameras;
//using namespace aslam::python;

// template<typename DERIVED_Y>
// void distort(const Eigen::MatrixBase<DERIVED_Y> & y) const;
template<typename D>
Eigen::Vector2d distort1(const D * distortion, Eigen::Vector2d k) {
  distortion->distort(k);
  return k;
}

// template<typename DERIVED_Y, typename DERIVED_JY>
// void distort(const Eigen::MatrixBase<DERIVED_Y> & y, const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

template<typename D>
boost::python::tuple distort2(const D * distortion, Eigen::Vector2d k) {
  Eigen::Matrix2d J;
  distortion->distort(k, J);
  return boost::python::make_tuple(k, J);
}

// template<typename DERIVED>
// void undistort(const Eigen::MatrixBase<DERIVED> & y) const;
template<typename D>
Eigen::Vector2d undistort1(const D * distortion, Eigen::Vector2d k) {

  distortion->undistort(k);
  return k;
}

// template<typename DERIVED, typename DERIVED_JY>
// void undistort(const Eigen::MatrixBase<DERIVED> & y,
// 					const Eigen::MatrixBase<DERIVED_JY> & outJy) const;
template<typename D>
boost::python::tuple undistort2(const D * distortion, Eigen::Vector2d k) {
  Eigen::Matrix2d J;
  distortion->undistort(k, J);
  return boost::python::make_tuple(k, J);
}

// template<typename DERIVED_Y, typename DERIVED_JD>
// void distortParameterJacobian(const Eigen::MatrixBase<DERIVED_Y> & imageY, const Eigen::MatrixBase<DERIVED_JD> & outJd) const;
template<typename D>
Eigen::MatrixXd distortParameterJacobian(const D * distortion,
                                         Eigen::Vector2d imageY) {
  Eigen::MatrixXd J;
  distortion->distortParameterJacobian(imageY, J);
  return J;
}

template<typename C>
Eigen::VectorXd e2k(const C * camera, Eigen::Vector3d const & p) {
  Eigen::VectorXd k;
  camera->euclideanToKeypoint(p, k);
  return k;
}

template<typename C>
boost::python::tuple e2kJp(const C * camera, Eigen::Vector3d const & p) {
  Eigen::MatrixXd Jp;
  Eigen::VectorXd k;
  bool isValid = camera->euclideanToKeypoint(p, k, Jp);
  return boost::python::make_tuple(k, Jp, isValid);
}

template<typename C>
Eigen::VectorXd eh2k(const C * camera, Eigen::Vector4d const & p) {
  Eigen::VectorXd k;
  camera->homogeneousToKeypoint(p, k);
  return k;
}

template<typename C>
boost::python::tuple eh2kJp(const C * camera, Eigen::Vector4d const & p) {
  Eigen::MatrixXd Jp;
  Eigen::VectorXd k;
  bool valid = camera->homogeneousToKeypoint(p, k, Jp);
  return boost::python::make_tuple(k, Jp, valid);
}

template<typename C>
Eigen::Vector3d k2e(const C * camera, Eigen::VectorXd const & k) {
  Eigen::Vector3d p;
  camera->keypointToEuclidean(k, p);
  return p;
}

template<typename C>
boost::python::tuple k2eJk(const C * camera, Eigen::VectorXd const & k) {
  Eigen::MatrixXd Jk;
  Eigen::Vector3d p;
  bool valid = camera->keypointToEuclidean(k, p, Jk);
  return boost::python::make_tuple(p, Jk, valid);
}

template<typename C>
Eigen::Vector4d k2eh(const C * camera, Eigen::VectorXd const & k) {
  Eigen::VectorXd ph;
  camera->keypointToHomogeneous(k, ph);
  return ph;
}

template<typename C>
boost::python::tuple k2ehJk(const C * camera, Eigen::VectorXd const & k) {
  Eigen::MatrixXd Jk;
  Eigen::Vector4d p;
  bool valid = camera->keypointToHomogeneous(k, p, Jk);
  return boost::python::make_tuple(p, Jk, valid);
}

template<typename T>
Eigen::MatrixXd getParameters(T * D) {

  Eigen::MatrixXd P;
  D->getParameters(P);
  return P;

}

// template <typename C>
// void exportGenericProjectionDesignVariable(std::string name) {

// 	// export the corresponding DV
// 	exportDesignVariableAdapter< C >( name + "DesignVariable");

// }

template<typename C, typename T>
void exportGenericProjectionFunctions(T & proj) {
  proj.def("keypointDimension", &C::keypointDimension,
           "Get the dimension of the keypoint type");
  proj.def(
      "isProjectionInvertible",
      &C::isProjectionInvertible,
      "Is the sensor model invertible? Ususally this is only true for a range/bearing sensor.");
  proj.def(
      "createRandomKeypoint",
      &C::createRandomKeypoint,
      "Create a valid, random keypoint. This is useful for unit testing and experiments.");
  proj.def(
      "createRandomVisiblePoint",
      &C::createRandomVisiblePoint,
      "Create a valid point in space visible by the camera.\np = createRandomVisiblePoint(depth).");
  proj.def(
      "euclideanToKeypoint", &e2k<C>,
      "Map a 3x1 Euclidean point to a keypoint.\nk = euclideanToKeypoint(p)");
  proj.def(
      "euclideanToKeypointJp",
      &e2kJp<C>,
      "Map a 3x1 Euclidean point to a keypoint and get the Jacobian of the mapping with respect to small changes in the point.\n(k, Jp) = euclideanToKeypoint(p)");
  proj.def(
      "homogeneousToKeypoint",
      &eh2k<C>,
      "Map a 4x1 homogeneous Euclidean point to a keypoint.\nk = euclideanToKeypoint(p)");
  proj.def(
      "homogeneousToKeypointJp",
      &eh2kJp<C>,
      "Map a 4x1 homogeneous Euclidean point to a keypoint and get the Jacobian of the mapping with respect to small changes in the point.\n(k, Jp) = homogeneousToKeypointJp(p)");
  proj.def(
      "keypointToHomogeneous",
      &k2eh<C>,
      "Map a keypoint to a 4x1 homogeneous Euclidean point.\np = keypointToHomogeneous(k)");
  proj.def(
      "keypointToHomogeneousJk",
      &eh2kJp<C>,
      "Map a keypoint to a 4x1 homogeneous Euclidean point and get the Jacobian of the mapping with respect to small changes in the keypoint.\n(p, Jk) = keypointToHomogeneousJk(k)");
  proj.def(
      "keypointToEuclidean", &k2e<C>,
      "Map a keypoint to a 3x1 Euclidean point.\np = keypointToEuclidean(k)");
  proj.def(
      "keypointToEuclideanJk",
      &e2kJp<C>,
      "Map a keypoint to a 3x1 Euclidean point and get the Jacobian of the mapping with respect to small changes in the keypoint.\n(p, Jk) = keypointToEuclideanJk(k)");
  proj.def("isValid", &C::template isValid<Eigen::VectorXd>);
  proj.def("setParameters", &C::setParameters,
           "Set the Parameter Vector/Matrix");
  proj.def("getParameters", &getParameters<C>,
           "Get the Parameter Vector/Matrix");
}

template<typename C, typename T>
void exportGenericDistortionFunctions(T & dist) {
  dist.def("distort", &distort1<C>, "Apply distortion to a keypoint");
  dist.def(
      "distortJd", &distort2<C>,
      "Apply distortion and get the Jacobian with respect to the input point");
  dist.def("undistort", &undistort1<C>, "Undistort a keypoint");
  dist.def(
      "undistort",
      &undistort2<C>,
      "Undistort a keypoint and get the Jacobian with respect to small chages in the input (distorted) keypoint.");
  dist.def(
      "distortParameterJacobian",
      &distortParameterJacobian<C>,
      "Get the Jacobian of the distortion operation with respect to small changes in the distortion parameters");
  dist.def("setParameters", &C::setParameters,
           "Set the Parameter Vector/Matrix");
  dist.def("getParameters", &getParameters<C>,
           "Get the Parameter Vector/Matrix");
}

void exportFovDistortionFunctions() {
  class_<FovDistortion, boost::shared_ptr<FovDistortion> > distortion("FovDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<FovDistortion> >();

  exportGenericDistortionFunctions<FovDistortion>(distortion);
  distortion.def(init<double>(("FovDistortion(double w)")));
  distortion.def("w", &FovDistortion::w);
}

void exportRadialTangentialDistortionFunctions() {

  class_<RadialTangentialDistortion,
      boost::shared_ptr<RadialTangentialDistortion> > rtDistortion(
      "RadialTangentialDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<RadialTangentialDistortion> >();

  exportGenericDistortionFunctions<RadialTangentialDistortion>(rtDistortion);

  rtDistortion.def(
      init<double, double, double, double>(
          ("RadialTangentialDistortion(double k1, double k2, double p1, double p2)")));
  rtDistortion.def("k1", &RadialTangentialDistortion::k1);
  rtDistortion.def("k2", &RadialTangentialDistortion::k2);
  rtDistortion.def("p1", &RadialTangentialDistortion::p1);
  rtDistortion.def("p2", &RadialTangentialDistortion::p2);
  //rtDistortion.def("getLinesPack", &RadialTangentialDistortion::getLinesPack);
  //rtDistortion.def("distortionError", &RadialTangentialDistortion::distortionError);

  //exportGenericProjectionDesignVariable<RadialTangentialDistortion>("RadialTangentialDistortion");
}

template<typename D>
void exportOmniProjection(std::string name) {

  D & (OmniProjection<D>::*omnidistortion)() = &OmniProjection<D>::distortion;

  class_<OmniProjection<D>, boost::shared_ptr<OmniProjection<D> > > omniProjection(
      name.c_str(), init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<OmniProjection<D> > >();

  omniProjection.def(init<>((name + "(distortion_t distortion)").c_str())).def(
      init<double, double, double, double, double, int, int, D>(
          (name
              + "(double xi, double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV, distortion_t distortion)")
              .c_str())).def(
      init<double, double, double, double, double, int, int>(
          (name
              + "(double xi, double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV)")
              .c_str()))
  /// \brief the xi parameter that controls the spherical projection.
      .def("xi", &OmniProjection<D>::xi)
  /// \brief The horizontal focal length in pixels.
      .def("fu", &OmniProjection<D>::fu)
  /// \brief The vertical focal length in pixels.
      .def("fv", &OmniProjection<D>::fv)
  /// \brief The horizontal image center in pixels.
      .def("cu", &OmniProjection<D>::cu)
  /// \brief The vertical image center in pixels.
      .def("cv", &OmniProjection<D>::cv)
  /// \brief The horizontal resolution in pixels.
      .def("ru", &OmniProjection<D>::ru)
  /// \brief The vertical resolution in pixels.
      .def("rv", &OmniProjection<D>::rv).def("focalLengthCol",
                                             &OmniProjection<D>::focalLengthCol)
      .def("focalLengthRow", &OmniProjection<D>::focalLengthRow).def(
      "opticalCenterCol", &OmniProjection<D>::opticalCenterCol).def(
      "opticalCenterRow", &OmniProjection<D>::opticalCenterRow).def(
      "distortion", omnidistortion, return_internal_reference<>()).def(
      "setDistortion", &OmniProjection<D>::setDistortion).def_pickle(
      sm::python::pickle_suite<OmniProjection<D> >());
  exportGenericProjectionFunctions<OmniProjection<D> >(omniProjection);
  //exportGenericProjectionDesignVariable< OmniProjection<D> >(name);

}

template<typename D>
void exportExtendedUnifiedProjection(std::string name) {

  D & (ExtendedUnifiedProjection<D>::*distortion)() = &ExtendedUnifiedProjection<D>::distortion;

  class_<ExtendedUnifiedProjection<D>, boost::shared_ptr<ExtendedUnifiedProjection<D> > > extendedUnifiedProjection(
      name.c_str(), init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<ExtendedUnifiedProjection<D> > >();

  extendedUnifiedProjection.def(init<>((name + "(distortion_t distortion)").c_str())).def(
      init<double, double, double, double, double, double, int, int, D>(
          (name
              + "(double alpha, double beta, double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV, distortion_t distortion)")
              .c_str())).def(
      init<double, double, double, double, double, double, int, int>(
          (name
              + "(double alpha, double beta, double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV)")
              .c_str()))
  /// \brief the alpha parameter that relates ellipsoid and pinhole projection.
      .def("alpha", &ExtendedUnifiedProjection<D>::alpha)
  /// \brief the beta parameter that controls the ellipsoid shape.
      .def("beta", &ExtendedUnifiedProjection<D>::beta)
  /// \brief The horizontal focal length in pixels.
      .def("fu", &ExtendedUnifiedProjection<D>::fu)
  /// \brief The vertical focal length in pixels.
      .def("fv", &ExtendedUnifiedProjection<D>::fv)
  /// \brief The horizontal image center in pixels.
      .def("cu", &ExtendedUnifiedProjection<D>::cu)
  /// \brief The vertical image center in pixels.
      .def("cv", &ExtendedUnifiedProjection<D>::cv)
  /// \brief The horizontal resolution in pixels.
      .def("ru", &ExtendedUnifiedProjection<D>::ru)
  /// \brief The vertical resolution in pixels.
      .def("rv", &ExtendedUnifiedProjection<D>::rv).def("focalLengthCol",
                                             &ExtendedUnifiedProjection<D>::focalLengthCol)
      .def("focalLengthRow", &ExtendedUnifiedProjection<D>::focalLengthRow).def(
      "opticalCenterCol", &ExtendedUnifiedProjection<D>::opticalCenterCol).def(
      "opticalCenterRow", &ExtendedUnifiedProjection<D>::opticalCenterRow).def(
      "distortion", distortion, return_internal_reference<>()).def(
      "setDistortion", &ExtendedUnifiedProjection<D>::setDistortion).def_pickle(
      sm::python::pickle_suite<ExtendedUnifiedProjection<D> >());
  exportGenericProjectionFunctions<ExtendedUnifiedProjection<D> >(extendedUnifiedProjection);
  //exportGenericProjectionDesignVariable< ExtendedUnifiedProjection<D> >(name);

}

template<typename D>
void exportDoubleSphereProjection(std::string name) {

  D & (DoubleSphereProjection<D>::*distortion)() = &DoubleSphereProjection<D>::distortion;

  class_<DoubleSphereProjection<D>, boost::shared_ptr<DoubleSphereProjection<D> > > doubleSphereProjection(
      name.c_str(), init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<DoubleSphereProjection<D> > >();

  doubleSphereProjection.def(init<>((name + "(distortion_t distortion)").c_str())).def(
      init<double, double, double, double, double, double, int, int, D>(
          (name
              + "(double xi, double alpha, double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV, distortion_t distortion)")
              .c_str())).def(
      init<double, double, double, double, double, double, int, int>(
          (name
              + "(double xi, double alpha, double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV)")
              .c_str()))
  /// \brief the xi parameter corresponding to distance between spheres.
      .def("xi", &DoubleSphereProjection<D>::xi)
  /// \brief the alpha parameter that relates second sphere and pinhole projection.
      .def("alpha", &DoubleSphereProjection<D>::alpha)
  /// \brief The horizontal focal length in pixels.
      .def("fu", &DoubleSphereProjection<D>::fu)
  /// \brief The vertical focal length in pixels.
      .def("fv", &DoubleSphereProjection<D>::fv)
  /// \brief The horizontal image center in pixels.
      .def("cu", &DoubleSphereProjection<D>::cu)
  /// \brief The vertical image center in pixels.
      .def("cv", &DoubleSphereProjection<D>::cv)
  /// \brief The horizontal resolution in pixels.
      .def("ru", &DoubleSphereProjection<D>::ru)
  /// \brief The vertical resolution in pixels.
      .def("rv", &DoubleSphereProjection<D>::rv).def("focalLengthCol",
                                             &DoubleSphereProjection<D>::focalLengthCol)
      .def("focalLengthRow", &DoubleSphereProjection<D>::focalLengthRow).def(
      "opticalCenterCol", &DoubleSphereProjection<D>::opticalCenterCol).def(
      "opticalCenterRow", &DoubleSphereProjection<D>::opticalCenterRow).def(
      "distortion", distortion, return_internal_reference<>()).def(
      "setDistortion", &DoubleSphereProjection<D>::setDistortion).def_pickle(
      sm::python::pickle_suite<DoubleSphereProjection<D> >());
  exportGenericProjectionFunctions<DoubleSphereProjection<D> >(doubleSphereProjection);
  //exportGenericProjectionDesignVariable< DoubleSphereProjection<D> >(name);

}

template<typename D>
void exportPinholeProjection(std::string name) {

  typename PinholeProjection<D>::distortion_t & (PinholeProjection<D>::*distortion1)() = &PinholeProjection<D>::distortion;

  class_<PinholeProjection<D>, boost::shared_ptr<PinholeProjection<D> > > pinholeProjection(
      name.c_str(), init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<PinholeProjection<D> > >();

  pinholeProjection.def(init<>((name + "(distortion_t distortion)").c_str()))
      .def(
      init<double, double, double, double, int, int, D>(
          (name
              + "(double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV, distortion_t distortion)")
              .c_str())).def(
      init<double, double, double, double, int, int>(
          (name
              + "(double focalLengthU, double focalLengthV, double imageCenterU, double imageCenterV, int resolutionU, int resolutionV)")
              .c_str()))
  /// \brief The horizontal focal length in pixels.
      .def("fu", &PinholeProjection<D>::fu)
  /// \brief The vertical focal length in pixels.
      .def("fv", &PinholeProjection<D>::fv)
  /// \brief The horizontal image center in pixels.
      .def("cu", &PinholeProjection<D>::cu)
  /// \brief The vertical image center in pixels.
      .def("cv", &PinholeProjection<D>::cv)
  /// \brief The horizontal resolution in pixels.
      .def("ru", &PinholeProjection<D>::ru)
  /// \brief The vertical resolution in pixels.
      .def("rv", &PinholeProjection<D>::rv).def(
      "focalLengthCol", &PinholeProjection<D>::focalLengthCol).def(
      "focalLengthRow", &PinholeProjection<D>::focalLengthRow).def(
      "opticalCenterCol", &PinholeProjection<D>::opticalCenterCol).def(
      "opticalCenterRow", &PinholeProjection<D>::opticalCenterRow)
  //.def("distortion",&PinholeProjection<D>::distortion, return_internal_reference<>())
      .def("distortion", distortion1, return_internal_reference<>()).def(
      "setDistortion", &PinholeProjection<D>::setDistortion).def_pickle(
      sm::python::pickle_suite<PinholeProjection<D> >());
  exportGenericProjectionFunctions<PinholeProjection<D> >(pinholeProjection);
  //exportGenericProjectionDesignVariable< PinholeProjection<D> >(name);

}

void exportCameraProjections() {
  using namespace boost::python;
  using namespace aslam::cameras;
  //using namespace aslam::python;

  class_<NoDistortion, boost::shared_ptr<NoDistortion> > noDistortion(
      "NoDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<NoDistortion> >();

  exportGenericDistortionFunctions<NoDistortion>(noDistortion);

  class_<EquidistantDistortion, boost::shared_ptr<EquidistantDistortion> > equidistantDistortion(
      "EquidistantDistortion", init<>());
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<EquidistantDistortion> >();

  equidistantDistortion.def(init<double, double, double, double>());
  exportGenericDistortionFunctions<EquidistantDistortion>(
      equidistantDistortion);
  //exportGenericProjectionDesignVariable<NoDistortion>("NoDistortion");

  exportRadialTangentialDistortionFunctions();
  exportFovDistortionFunctions();

  exportPinholeProjection<NoDistortion>("PinholeProjection");
  exportPinholeProjection<RadialTangentialDistortion>(
      "DistortedPinholeProjection");
  exportPinholeProjection<EquidistantDistortion>(
      "EquidistantPinholeProjection");
  exportPinholeProjection<FovDistortion>(
        "FovPinholeProjection");

  exportOmniProjection<NoDistortion>("OmniProjection");
  exportOmniProjection<RadialTangentialDistortion>("DistortedOmniProjection");
  exportOmniProjection<FovDistortion>("FovOmniProjection");

  exportExtendedUnifiedProjection<NoDistortion>("ExtendedUnifiedProjection");

  exportDoubleSphereProjection<NoDistortion>("DoubleSphereProjection");


  // distortion:
  // exportAPrioriInformationError<aslam::backend::DesignVariableAdapter< RadialTangentialDistortion > >("RadialTangentialDistortionAPrioriInformationError");
  // exportAPrioriInformationError<aslam::backend::DesignVariableAdapter< PinholeProjection<RadialTangentialDistortion> > >("DistortedPinholeProjectionAPrioriInformationError");
  // exportAPrioriInformationError<aslam::backend::DesignVariableAdapter< OmniProjection<RadialTangentialDistortion> > >("DistortedOmniProjectionAPrioriInformationError");

}
