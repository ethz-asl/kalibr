#ifndef ASLAM_CAMERAS_TRIANGULATION_HPP
#define ASLAM_CAMERAS_TRIANGULATION_HPP

#include <Eigen/Core>

namespace aslam {
namespace cameras {

/// \brief Closed form linear triangulation 
///
/// Returns the point that is in between the closest point
/// between the two lines. The gap is the distance between
/// the two lines. s0 and s1 are the distance along each ray
/// to the closest point
void triangulate(const Eigen::Vector3d & point0, const Eigen::Vector3d & ray0,
                 const Eigen::Vector3d & point1, const Eigen::Vector3d & ray1,
                 Eigen::Vector3d & outTriangulatedPoint, double & outGap,
                 double & outS0, double & outS1);
}  // namespace cameras    
}  // namespace aslam

#endif /* ASLAM_CAMERAS_TRIANGULATION_HPP */
