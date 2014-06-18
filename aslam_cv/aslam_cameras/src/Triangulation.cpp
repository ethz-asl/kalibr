#include <aslam/cameras/Triangulation.hpp>
#include <Eigen/Dense>

namespace aslam {
namespace cameras {

void triangulate(const Eigen::Vector3d & point1, const Eigen::Vector3d & ray1,
                 const Eigen::Vector3d & point2, const Eigen::Vector3d & ray2,
                 Eigen::Vector3d & outTriangulatedPoint, double & outGap,
                 double & outS1, double & outS2) {

  Eigen::Vector3d t12 = point2 - point1;

  Eigen::Vector2d b;
  b[0] = t12.dot(ray1);
  b[1] = t12.dot(ray2);
  Eigen::Matrix2d A;
  A(0, 0) = ray1.dot(ray1);
  A(1, 0) = ray1.dot(ray2);
  A(0, 1) = -A(1, 0);
  A(1, 1) = -ray2.dot(ray2);
  Eigen::Vector2d lambda = A.inverse() * b;
  Eigen::Vector3d xm = point1 + lambda[0] * ray1;
  Eigen::Vector3d xn = point2 + lambda[1] * ray2;
  t12 = (xm - xn);

  outGap = t12.norm();
  outTriangulatedPoint = xn + 0.5 * t12;
  outS1 = lambda[0];
  outS2 = lambda[1];

}

}  // namespace cameras
}  // namespace aslam
