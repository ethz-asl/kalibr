#include <aslam/BackProjection.hpp>

namespace aslam {

/// \brief initialize both to zero with zero uncertainty
BackProjection::BackProjection() {
}

/// \brief the view origin is set to zero
BackProjection::BackProjection(const Eigen::Vector3d & ray)
    : ray(ray) {
}

/// \brief initialize without uncertainty
BackProjection::BackProjection(const Eigen::Vector3d & ray,
                               const Eigen::Vector3d & viewOrigin)
    : ray(ray),
      viewOrigin(viewOrigin) {
}

BackProjection::~BackProjection() {
}

/// \brief initialize both to zero with zero uncertainty
UncertainBackProjection::UncertainBackProjection() {
}

/// \brief the view origin is set to zero
UncertainBackProjection::UncertainBackProjection(const Eigen::Vector3d & ray)
    : ray(ray) {
}

/// \brief the view origin is set to zero
UncertainBackProjection::UncertainBackProjection(
    const sm::kinematics::UncertainVector3 & ray)
    : ray(ray) {
}

/// \brief initialize without uncertainty
UncertainBackProjection::UncertainBackProjection(
    const Eigen::Vector3d & ray, const Eigen::Vector3d & viewOrigin)
    : ray(ray),
      viewOrigin(viewOrigin) {
}

UncertainBackProjection::UncertainBackProjection(
    const sm::kinematics::UncertainVector3 & ray,
    const Eigen::Vector3d & viewOrigin)
    : ray(ray),
      viewOrigin(viewOrigin) {
}

/// \brief full initialization with uncertainty.
UncertainBackProjection::UncertainBackProjection(
    const sm::kinematics::UncertainVector3 & ray,
    const sm::kinematics::UncertainVector3 & viewOrigin)
    : ray(ray),
      viewOrigin(viewOrigin) {
}

UncertainBackProjection::~UncertainBackProjection() {
}

}  // namespace aslam
