#include <bsplines/BSplinePose.hpp>

#include <numpy_eigen/boost_python_headers.hpp>


boost::python::tuple orientationAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi I;
  Eigen::Matrix3d C = bsp->orientationAndJacobian(t, &J, &I);

  return boost::python::make_tuple(C,J,I);
}

boost::python::tuple inverseOrientationAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi I;
  Eigen::Matrix3d C = bsp->inverseOrientationAndJacobian(t, &J, &I);

  return boost::python::make_tuple(C,J,I);
}


boost::python::tuple transformationAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi I;
  Eigen::Matrix4d T = bsp->transformationAndJacobian(t, &J, &I);

  return boost::python::make_tuple(T,J,I);
}

boost::python::tuple inverseTransformationAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi I;
  Eigen::Matrix4d T = bsp->inverseTransformationAndJacobian(t, &J, &I);

  return boost::python::make_tuple(T,J,I);
}


boost::python::tuple angularVelocityAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi coefficientIndices;
  Eigen::Vector3d omega = bsp->angularVelocityAndJacobian(t, &J, &coefficientIndices);
  //std::cout << "J\n" << J << std::endl;
  return boost::python::make_tuple(omega,J,coefficientIndices);
}

boost::python::tuple angularVelocityBodyFrameAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi coefficientIndices;
  Eigen::Vector3d omega = bsp->angularVelocityBodyFrameAndJacobian(t, &J, &coefficientIndices);
  //std::cout << "J\n" << J << std::endl;
  return boost::python::make_tuple(omega,J,coefficientIndices);
}


boost::python::tuple linearAccelerationAndVelocityWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi coefficientIndices;
  Eigen::Vector3d a = bsp->linearAccelerationAndJacobian(t, &J, &coefficientIndices);
  return boost::python::make_tuple(a,J,coefficientIndices);
}

boost::python::tuple angularAccelerationAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi coefficientIndices;
  Eigen::Vector3d omega = bsp->angularAccelerationAndJacobian(t, &J, &coefficientIndices);
  return boost::python::make_tuple(omega,J,coefficientIndices);
}

boost::python::tuple angularAccelerationBodyFrameAndJacobianWrapper(const bsplines::BSplinePose * bsp, double t)
{
  Eigen::MatrixXd J;
  Eigen::VectorXi coefficientIndices;
  Eigen::Vector3d omega = bsp->angularAccelerationBodyFrameAndJacobian(t, &J, &coefficientIndices);
  return boost::python::make_tuple(omega,J,coefficientIndices);
}


void import_bspline_pose_python()
{
  using namespace bsplines;
  using namespace boost::python;
  using namespace sm::kinematics;


  // This initialization actually works! boost::python is amazing.
  class_<BSplinePose, bases<BSpline> >("BSplinePose", init<int, const RotationalKinematics::Ptr &>())
    .def("transformation",&BSplinePose::transformation)
    .def("inverseTransformation",&BSplinePose::inverseTransformation)
    .def("initPoseSpline", &BSplinePose::initPoseSpline)
    .def("initPoseSpline2", &BSplinePose::initPoseSpline2)
    .def("initPoseSpline3", &BSplinePose::initPoseSpline3)
    .def("initPoseSplineSparse", &BSplinePose::initPoseSplineSparse)
    .def("initPoseSplineSparseKnots", &BSplinePose::initSplineSparseKnots)
    .def("addPoseSegment", &BSplinePose::addPoseSegment)
    .def("addPoseSegment2", &BSplinePose::addPoseSegment2)
    .def("curveValueToTransformation", &BSplinePose::curveValueToTransformation)
    .def("transformationToCurveValue", &BSplinePose::transformationToCurveValue)
    .def("transformationAndJacobian",  &transformationAndJacobianWrapper)
    .def("orientation",  &BSplinePose::orientation)
    .def("orientationAndJacobian",  &orientationAndJacobianWrapper)
    .def("inverseOrientation",  &BSplinePose::inverseOrientation)
    .def("inverseOrientationAndJacobian",  &inverseOrientationAndJacobianWrapper)
    .def("inverseTransformationAndJacobian",  &inverseTransformationAndJacobianWrapper)
    .def("position", &BSplinePose::position)
    .def("linearVelocity", &BSplinePose::linearVelocity)
    .def("linearAcceleration", &BSplinePose::linearAcceleration)
    .def("angularVelocity", &BSplinePose::angularVelocity)
    .def("angularVelocityAndJacobian", &angularVelocityAndJacobianWrapper)
    .def("angularVelocityBodyFrame", &BSplinePose::angularVelocityBodyFrame)
    .def("angularVelocityBodyFrameAndJacobian", &angularVelocityBodyFrameAndJacobianWrapper)
    .def("angularAccelerationBodyFrame", &BSplinePose::angularAccelerationBodyFrame)
    .def("angularAccelerationBodyFrameAndJacobian", &angularAccelerationBodyFrameAndJacobianWrapper)
    .def("rotation", &BSplinePose::rotation);
  //.def("", &BSplinePose::, "")

}

