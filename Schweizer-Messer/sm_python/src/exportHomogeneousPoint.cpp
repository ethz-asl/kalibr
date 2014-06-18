#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/HomogeneousPoint.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>

using namespace boost::python;
using namespace sm::kinematics;

void exportHomogeneousPoint()
{

  typedef Eigen::Matrix3d euclidean_jacobian_t;
  typedef Eigen::Matrix<double,4,3> homogeneous_jacobian_t;

  class_<HomogeneousPoint, boost::shared_ptr<HomogeneousPoint> >("HomogeneousPoint", init<>())
    .def(init<const Eigen::Vector3d &>())
    .def(init<const Eigen::Vector4d &>())
    .def("toEuclidean",&HomogeneousPoint::toEuclidean)
      // Eigen::Vector3d toEuclideanAndJacobian(euclidean_jacobian_t & J) const;
    .def("toHomogeneous",&HomogeneousPoint::toHomogeneous, return_value_policy<copy_const_reference>())
      // const Eigen::Vector4d & toHomogeneousAndJacobian(homogeneous_jacobian_t & J) const;
    .def(self + self)
    .def(self + UncertainHomogeneousPoint())
    .def(self - self)
    .def(self - UncertainHomogeneousPoint())
    .def("setRandom", &HomogeneousPoint::setRandom)
    .def("setZero", &HomogeneousPoint::setZero)
    .def("normalize", &HomogeneousPoint::normalize)
    ;

  class_<UncertainHomogeneousPoint, boost::shared_ptr<UncertainHomogeneousPoint>, bases<HomogeneousPoint> >("UncertainHomogeneousPoint", init<>())
    .def(init<const Eigen::Vector3d &>())
    .def(init<const Eigen::Vector4d &>())
    .def(init<const HomogeneousPoint &>())
    .def(init<const Eigen::Vector3d &, const Eigen::Matrix3d &>())
    .def(init<const Eigen::Vector4d &, const Eigen::Matrix4d &>())
    .def(init<const HomogeneousPoint &, const Eigen::Matrix4d &>())
    .def(init<const Eigen::Vector4d &, const Eigen::Matrix3d &>())
    .def(init<const HomogeneousPoint &, const Eigen::Matrix3d &>())
    .def("setRandom", &UncertainHomogeneousPoint::setRandom)
    .def(self + HomogeneousPoint())
    .def(self + UncertainHomogeneousPoint())
    .def(self - HomogeneousPoint())
    .def(self - UncertainHomogeneousPoint())
    .def("U4", &UncertainHomogeneousPoint::U4, return_value_policy<copy_const_reference>())
    .def("U3", &UncertainHomogeneousPoint::U3)
    .def("U_av_form", &UncertainHomogeneousPoint::U_av_form)
    .def("setU", &UncertainHomogeneousPoint::setU)
    .def("setUOplus", &UncertainHomogeneousPoint::setUOplus)
    .def("normalize", &UncertainHomogeneousPoint::normalize)
    ;

}
