#ifndef ASLAM_PYTHON_EXPORT_FRAME_HPP
#define ASLAM_PYTHON_EXPORT_FRAME_HPP
#include <sstream>
#include <aslam/Frame.hpp>
//#include <aslam/backend/ReprojectionError.hpp>
//#include <aslam/backend/CovarianceReprojectionError.hpp>
//#include <aslam/backend/SimpleReprojectionError.hpp>
//#include <aslam/backend/ReprojectionIntrinsicsError.hpp>
//#include <aslam/backend/HomogeneousExpression.hpp>
//#include <aslam/backend/CameraDesignVariable.hpp>
//#include <aslam/backend/Scalar.hpp>

namespace aslam {
namespace python {

template<int D>
void exportKeypoint() {
  using namespace boost::python;
  using namespace aslam;

  typedef aslam::Keypoint<D> keypoint_t;
  typedef DescriptorBase descriptor_t;

  std::stringstream str;
  str << "Keypoint" << D;

  class_<keypoint_t, bases<KeypointBase> >(str.str().c_str(), init<>()).def(
      "measurement", &keypoint_t::measurement,
      return_value_policy<copy_const_reference>()).def(
      "y", &keypoint_t::y, return_value_policy<copy_const_reference>()).def(
      "setMeasurement", &keypoint_t::setMeasurement).def("octave",
                                                         &keypoint_t::octave)
      .def("setOctave", &keypoint_t::setOctave).def(
      "inverseMeasurementCovariance", &keypoint_t::inverseMeasurementCovariance,
      return_value_policy<copy_const_reference>()).def(
      "invR", &keypoint_t::invR, return_value_policy<copy_const_reference>())
      .def("setInverseMeasurementCovariance",
           &keypoint_t::setInverseMeasurementCovariance)

  // /// \brief get the landmark
  // virtual const sm::kinematics::UncertainHomogeneousPoint * landmark() const;
      .def("landmark", &keypoint_t::landmark, return_internal_reference<>())
  // /// \brief set the landmark
  // virtual void setLandmark(const sm::kinematics::UncertainHomogeneousPoint & landmark);

  // /// \brief get the landmark id
  // virtual const LandmarkId & landmarkId() const;

  // /// \brief set the landmark id
  // virtual void setLandmarkId(const LandmarkId & landmarkId);

  // /// \brief is the landmark initialized
  // virtual bool isLandmarkInitialized() const;

  // /// \brief reset and clear the landmark.
  // virtual void clearLandmark();

  // ///////////////////////////////////////////////////
  // // The back projection
  // ///////////////////////////////////////////////////

  // /// \brief set the back projection
  // virtual void setBackProjection(const sm::kinematics::UncertainVector3 & v);

  // /// \brief set the back projection
  // virtual void setBackProjectionVector(const Eigen::Vector3d & v);

  // /// \brief get the back projection pointer
  // virtual const sm::kinematics::UncertainVector3 * backProjection() const;

  // /// \brief is the back projection initialized?
  // virtual bool isBackProjectionSet() const;

  // /// \brief clear the back projection
  // virtual void clearBackProjection();

      ;

}

//     template<typename CAMERA_GEOMETRY_T>
//     void exportReprojectionError(const std::string & name)
//     {
//       using namespace boost::python;
//       using namespace aslam;
//       using namespace aslam::backend;
//       typedef CAMERA_GEOMETRY_T geometry_t;
//       typedef DescriptorBase descriptor_t;

// typedef Frame<geometry_t> frame_t;
//       typedef typename frame_t::keypoint_t keypoint_t;

//       class_< ReprojectionError<frame_t>, boost::shared_ptr<ReprojectionError<frame_t> >, bases< ErrorTerm > >( name.c_str(),
//     		  init<const frame_t * , int ,HomogeneousExpression, CameraDesignVariable<geometry_t> >( (name + "( frame, keypointIndex, homogeneousPointExpression, CameraDesignVariable)").c_str()) )
// 			;

//       class_< SimpleReprojectionError<frame_t>, boost::shared_ptr<SimpleReprojectionError<frame_t> >, bases< ErrorTerm > >( (name + "Simple").c_str(),
//     		  init<const frame_t * , int ,HomogeneousExpression >( (name + "Simple( frame, keypointIndex, homogeneousPointExpression )").c_str()) )
// 			;

//     }

//     template<typename CAMERA_GEOMETRY_T>
//     void exportCovarianceReprojectionError(const std::string & name)
//     {
//       using namespace boost::python;
//       using namespace aslam;
//       using namespace aslam::backend;
//       typedef CAMERA_GEOMETRY_T geometry_t;
//       typedef DescriptorBase descriptor_t;
//       typedef Frame<geometry_t> frame_t;
//       typedef typename frame_t::keypoint_t keypoint_t;

//       class_< CovarianceReprojectionError<frame_t>, boost::shared_ptr<CovarianceReprojectionError<frame_t> >, bases< ErrorTerm > >( name.c_str(),
//     		  init<const frame_t * , int ,HomogeneousExpression, CameraDesignVariable<geometry_t>, aslam::splines::BSplinePoseDesignVariable*, aslam::backend::Scalar* >( (name + "( frame, keypointIndex, homogeneousPointExpression, CameraDesignVariable, bsplineDesignVariable, lineDelayDv)").c_str()) )
// 			.def("observationTime", &CovarianceReprojectionError<frame_t>::observationTime)
// 			.def("covarianceMatrix",  &CovarianceReprojectionError<frame_t>::covarianceMatrix)
// 					;

//     }

// // export the optimizable intrinsics reprojection error:
// template<typename CAMERA_GEOMETRY_T, typename DESCRIPTOR_T>
// void exportReprojectionIntrinsicsError(const std::string & name)
// {
//   using namespace boost::python;
//   using namespace aslam;
//   using namespace aslam::backend;
//   typedef CAMERA_GEOMETRY_T geometry_t;
//   typedef DESCRIPTOR_T descriptor_t;
//   typedef Frame<geometry_t, descriptor_t> frame_t;
//   typedef typename frame_t::keypoint_t keypoint_t;

//   class_< ReprojectionIntrinsicsError<frame_t>, boost::shared_ptr<ReprojectionIntrinsicsError<frame_t> >, bases< ErrorTerm > >( name.c_str(), init<>() )
// .def(init<const frame_t * , int ,HomogeneousExpression >( (name + "( frame, keypointIndex, homogeneousPointExpression)").c_str()));

// }

template<typename CAMERA_GEOMETRY_T>
void exportFrame(const std::string & name) {
  using namespace boost::python;
  using namespace aslam;
  typedef CAMERA_GEOMETRY_T geometry_t;
  typedef DescriptorBase descriptor_t;
  typedef Frame<geometry_t> frame_t;
  typedef typename frame_t::keypoint_t keypoint_t;

  keypoint_t & (frame_t::*keypoint)(size_t) = &frame_t::keypoint;

  void (frame_t::*addKeypointPtr)(
      const keypoint_t & keypoint) = &frame_t::addKeypoint;

  class_<frame_t, boost::shared_ptr<frame_t>, bases<FrameBase> >(name.c_str(),
                                                                 init<>()).def(
      "geometry", &frame_t::geometryPtr).def("setGeometry",
                                             &frame_t::setGeometry).def(
      "keypoint", keypoint, return_internal_reference<>()).def("addKeypoint",
                                                               addKeypointPtr)
                                                               // void addKeypoints(const keypoint_vector_t & keypoints);
      ;

  //exportReprojectionError<geometry_t>(name + "ReprojectionError");

}

}  // namespace python
}  // namespace aslam

#endif /* ASLAM_PYTHON_EXPORT_FRAME_HPP */
