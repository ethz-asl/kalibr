#ifndef ASLAM_PYTHON_EXPORT_FRAME_HPP
#define ASLAM_PYTHON_EXPORT_FRAME_HPP
#include <sstream>
#include <aslam/Frame.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/CovarianceReprojectionError.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/Scalar.hpp>

namespace aslam {
namespace python {

template<typename CAMERA_GEOMETRY_T>
void exportReprojectionError(const std::string & camName) {
  std::string name = camName + "ReprojectionError";
  using namespace boost::python;
  using namespace aslam;
  using namespace aslam::backend;
  typedef CAMERA_GEOMETRY_T geometry_t;
  typedef typename ReprojectionError<geometry_t>::measurement_t measurement_t;
  typedef typename ReprojectionError<geometry_t>::inverse_covariance_t inverse_covariance_t;
  typedef DescriptorBase descriptor_t;

  typedef Frame<geometry_t> frame_t;
  typedef typename frame_t::keypoint_t keypoint_t;

  class_<ReprojectionError<geometry_t>,
      boost::shared_ptr<ReprojectionError<geometry_t> >, bases<ErrorTerm> >(
      (name).c_str(),
      init<const frame_t *, int, HomogeneousExpression,
          CameraDesignVariable<geometry_t> >(
          (name
              + "( frame, keypointIndex, homogeneousPointExpression, CameraDesignVariable)")
              .c_str())).def(
      init<typename ReprojectionError<geometry_t>::measurement_t,
          typename ReprojectionError<geometry_t>::inverse_covariance_t,
          HomogeneousExpression, CameraDesignVariable<geometry_t> >(
          (name + "( y, invR, homogeneousPointExpression, CameraDesignVariable)")
              .c_str())).def("updateMeasurement",
                             &ReprojectionError<geometry_t>::updateMeasurement)
      .def("updateMeasurementAndCovariance",
           &ReprojectionError<geometry_t>::updateMeasurementAndCovariance).def(
      "getMeasurement", &ReprojectionError<geometry_t>::getMeasurement).def(
      "getPredictedMeasurement",
      &ReprojectionError<geometry_t>::getPredictedMeasurement);

  class_<SimpleReprojectionError<frame_t>,
      boost::shared_ptr<SimpleReprojectionError<frame_t> >, bases<ErrorTerm> >(
      (name + "Simple").c_str(),
      init<const frame_t *, int, HomogeneousExpression>(
          (name + "Simple( frame, keypointIndex, homogeneousPointExpression )")
              .c_str())).def(
      init<const measurement_t &, const inverse_covariance_t &,
          HomogeneousExpression, const geometry_t &>(
          (name
              + "Simple( y, invR, homogeneousPointExpression, cameraGeometry )")
              .c_str()));

}

template<typename CAMERA_GEOMETRY_T>
void exportCovarianceReprojectionError(const std::string & camName)
{
  std::string name = camName + "ReprojectionErrorAdaptiveCovariance";
  using namespace boost::python;
  using namespace aslam;
  using namespace aslam::backend;
  typedef CAMERA_GEOMETRY_T geometry_t;
  typedef DescriptorBase descriptor_t;
  typedef Frame<geometry_t> frame_t;
  typedef typename frame_t::keypoint_t keypoint_t;

  class_<
    CovarianceReprojectionError<frame_t>,
    boost::shared_ptr<CovarianceReprojectionError<frame_t>
  >,bases< ErrorTerm > >(
      name.c_str(),
		  init<
        const frame_t *,
        int,
        HomogeneousExpression,
        CameraDesignVariable<geometry_t>,
        aslam::splines::BSplinePoseDesignVariable*
      >
      (
        (name + "( frame, keypointIndex, homogeneousPointExpression, CameraDesignVariable, bsplineDesignVariable)").c_str()
      )
    )
		.def("observationTime", &CovarianceReprojectionError<frame_t>::observationTime)
		.def("covarianceMap",  &CovarianceReprojectionError<frame_t>::covarianceMap)
		;

}

template<typename CAMERA_GEOMETRY_T>
void exportReprojectionErrors(const std::string & camName) {
  exportReprojectionError<CAMERA_GEOMETRY_T>(camName);
}

}  // namespace python
}  // namespace aslam

#endif /* ASLAM_PYTHON_EXPORT_FRAME_HPP */
