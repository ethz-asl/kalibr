#ifndef ASLAM_FRONTEND_UTILITIES_HPP
#define ASLAM_FRONTEND_UTILITIES_HPP

#include <aslam/BackProjection.hpp>

namespace aslam {

/// \brief triangulate and return true on success.
bool triangulate(const BackProjection & r1, const BackProjection & r2,
                 sm::kinematics::UncertainHomogeneousPoint & outPoint);

/// \brief triangulation that ignores uncertainty.
bool triangulateNoUncertainty(const BackProjection & r1,
                              const BackProjection & r2,
                              sm::kinematics::HomogeneousPoint & outPoint);

bool triangulateNoUncertainty(const Eigen::Vector3d & r1,
                              const Eigen::Vector3d v1,
                              const Eigen::Vector3d & r2,
                              const Eigen::Vector3d v2,
                              sm::kinematics::HomogeneousPoint & outPoint);

/// \brief evaluate the epipolar constriaint for the two back projections
double evaluateEpipolarConstraint(const BackProjection & r1,
                                  const BackProjection & r2);

/// \brief evaluate the epipolar constriaint for the two back projections
double evaluateEpipolarConstraintNoUncertainty(const BackProjection & r1,
                                               const BackProjection & r2);

double evaluateEpipolarConstraintNoUncertainty(const Eigen::Vector3d & r1,
                                               const Eigen::Vector3d v1,
                                               const Eigen::Vector3d & r2,
                                               const Eigen::Vector3d v2);

template<typename FRAME_T>
void doBackProjection(FRAME_T & frame);

template<typename FRAME_T>
void doBackProjectionWithUncertainty(FRAME_T & frame);

}  // namespace aslam

#include "implementation/utilities.hpp"

#endif /* ASLAM_FRONTEND_UTILITIES_HPP */
