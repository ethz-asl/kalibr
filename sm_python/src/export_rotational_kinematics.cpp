#include <numpy_eigen/boost_python_headers.hpp>

#include <sm/kinematics/RotationalKinematics.hpp>
#include <sm/kinematics/EulerAnglesZYX.hpp>
#include <sm/kinematics/EulerRodriguez.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>

// A wrapper that gets rid of the second parameter in the parametersToRotationMatrix() function.
template<typename ROTATION_TYPE>
Eigen::Matrix3d parametersToRotationMatrixWrapper(const ROTATION_TYPE * rt, const Eigen::Vector3d & parameters)
{
  return rt->parametersToRotationMatrix(parameters);
}

template<typename ROTATION_TYPE>
boost::python::tuple angularVelocityAndJacobianWrapper(const ROTATION_TYPE * rt, const Eigen::Vector3d & p, const Eigen::Vector3d pdot)
{
  Eigen::Matrix<double,3,6> J;
  Eigen::Vector3d omega = rt->angularVelocityAndJacobian(p,pdot,&J);

  return boost::python::make_tuple(omega,J);
}



template<typename ROTATION_TYPE>
class RotationalKinematicsPythonWrapper
{
public:
  typedef ROTATION_TYPE rotation_t;

  static void exportToPython(const std::string & className)
  {
    using namespace boost::python;

    class_<rotation_t, bases<sm::kinematics::RotationalKinematics> >(className.c_str())
      .def("parametersToRotationMatrix",&parametersToRotationMatrixWrapper<rotation_t>)
      .def("rotationMatrixToParameters",&rotation_t::rotationMatrixToParameters)
      .def("parametersToSMatrix",&rotation_t::parametersToSMatrix)
      .def("angularVelocityAndJacobian",&angularVelocityAndJacobianWrapper<rotation_t>);
  }
};


void import_rotational_kinematics_python()
{
  using namespace boost::python;
  // The no_init gets us a pure virtual base class.
  class_<sm::kinematics::RotationalKinematics, boost::noncopyable >("RotationalKinematics", no_init);

  RotationalKinematicsPythonWrapper<sm::kinematics::EulerAnglesZYX>::exportToPython("EulerAnglesZYX");
  RotationalKinematicsPythonWrapper<sm::kinematics::EulerAnglesYawPitchRoll>::exportToPython("EulerAnglesYawPitchRoll");
  RotationalKinematicsPythonWrapper<sm::kinematics::RotationVector>::exportToPython("RotationVector");
  RotationalKinematicsPythonWrapper<sm::kinematics::EulerRodriguez>::exportToPython("EulerRodriguez");
}
