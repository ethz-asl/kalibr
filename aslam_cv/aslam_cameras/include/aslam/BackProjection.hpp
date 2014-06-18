#ifndef ASLAM_BACK_PROJECTION_HPP
#define ASLAM_BACK_PROJECTION_HPP

#include <sm/kinematics/UncertainVector.hpp>
#include <sm/kinematics/UncertainTransformation.hpp>

namespace aslam {

struct BackProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief initialize both to zero with zero uncertainty
  BackProjection();

  virtual ~BackProjection();

  /// \brief the view origin is set to zero
  BackProjection(const Eigen::Vector3d & ray);

  /// \brief initialize without uncertainty
  BackProjection(const Eigen::Vector3d & ray,
                 const Eigen::Vector3d & viewOrigin);

  Eigen::Vector3d getRay() const {
    return ray;
  }
  void setRay(const Eigen::Vector3d & r) {
    ray = r;
  }
  Eigen::Vector3d getViewOrigin() const {
    return viewOrigin;
  }
  void setViewOrigin(const Eigen::Vector3d & v) {
    viewOrigin = v;
  }

  Eigen::Vector3d ray;
  Eigen::Vector3d viewOrigin;
};

struct UncertainBackProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief initialize both to zero with zero uncertainty
  UncertainBackProjection();

  virtual ~UncertainBackProjection();

  /// \brief the view origin is set to zero
  UncertainBackProjection(const Eigen::Vector3d & ray);

  /// \brief the view origin is set to zero
  UncertainBackProjection(const sm::kinematics::UncertainVector3 & ray);

  /// \brief initialize without uncertainty
  UncertainBackProjection(const Eigen::Vector3d & ray,
                          const Eigen::Vector3d & viewOrigin);

  UncertainBackProjection(const sm::kinematics::UncertainVector3 & ray,
                          const Eigen::Vector3d & viewOrigin);

  /// \brief full initialization with uncertainty.
  UncertainBackProjection(const sm::kinematics::UncertainVector3 & ray,
                          const sm::kinematics::UncertainVector3 & viewOrigin);

  sm::kinematics::UncertainVector3 ray;
  sm::kinematics::UncertainVector3 viewOrigin;
};

BackProjection operator*(const sm::kinematics::Transformation & T,
                         const BackProjection & bp);
UncertainBackProjection operator*(const sm::kinematics::Transformation & T,
                                  const UncertainBackProjection & bp);
UncertainBackProjection operator*(
    const sm::kinematics::UncertainTransformation & T,
    const BackProjection & bp);
UncertainBackProjection operator*(
    const sm::kinematics::UncertainTransformation & T,
    const UncertainBackProjection & bp);

}  // namespace aslam

#endif /* ASLAM_BACK_PROJECTION_HPP */
