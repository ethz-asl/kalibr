#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/transformations.hpp>


using namespace boost::python;
using namespace sm::kinematics;

tuple transformationAndJacobianTuple(Eigen::Matrix4d const & T_a_b, 
				     Eigen::Vector4d const & v_b)
    {
      Eigen::Vector4d out_v_a;
      Eigen::Matrix<double,4,6> out_B;
      transformationAndJacobian(T_a_b, v_b, out_v_a, out_B);
      return make_tuple(out_v_a, out_B);
      
    }

    tuple inverseTransformationAndJacobianTuple(Eigen::Matrix4d const & T_ba, 
						Eigen::Vector4d const & v_b)
    {
      Eigen::Vector4d out_v_a;
      Eigen::Matrix<double,4,6> out_B;
      inverseTransformationAndJacobian(T_ba, v_b, out_v_a, out_B);
      
      return make_tuple(out_v_a, out_B);
    }



  void export_transformations()
  {
    using namespace boost::python;
    using namespace sm;


    def("rt2Transform", rt2Transform, "creates a transformation matrix T_ba from the rotation matrix C_ba and the translation rho_b_ab");
    def("transform2C", transform2C, "returns the rotation matrix C_ba from the transformation matrix T_ba");
    def("transform2rho", transform2rho, "returns the translation p_b_ab from the transformation matrix T_ba");
    def("transform2rhoHomogeneous", transform2rhoHomogeneous, "returns the translation p_b_ab from the transformation matrix T_ba");
    def("boxPlus", boxPlus, "Builds the 4x4 box plus matrix from a 6x1 perturbation");
    def("boxMinus", boxMinus, "Builds the 6x6 box minus matrix from a 4x1 homogeneous point");
    def("boxTimes", boxTimes, "Builds the 6x6 box times matrix from a 4x4 transformation matrix");
    def("inverseTransform",inverseTransform, "Invert a 4x4 transformation matrix");

    Eigen::Matrix4d (*toTEuler1)(double, double, double, double, double, double) = &toTEuler;
    def("toTEuler",toTEuler1,"Create a 4x4 transformation matrix from 6 parameters");

    Eigen::Matrix4d (*toTEuler2)(Eigen::Matrix<double,6,1> const &) = &toTEuler;
    def("toTEuler",toTEuler2,"Create a 4x4 transformation matrix from a 6x1 column of parameters");
    def("fromTEuler",fromTEuler,"Create 6x1 column of parameters from a 4x4 transformation matrix");
    def("transformationAndJacobian",transformationAndJacobianTuple, "Transform a point and return the point and jacobian with respect to local perturbations of the transformation matrix. (v_a, H) = transformationAndJacobian(T_ab, v_b)");
    def("inverseTransformationAndJacobian",inverseTransformationAndJacobianTuple, "Transform a point (using the inverse of the supplied transformation matrix) and return the point and jacobian with respect to local perturbations of the transformation matrix. (v_b, H) = inverseTransformationAndJacobian(T_ab, v_a)");
  }
