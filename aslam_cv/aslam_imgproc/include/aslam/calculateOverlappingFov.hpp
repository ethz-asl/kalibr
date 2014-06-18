#ifndef ASLAM_CALCULATE_OVERLAPPING_FOV_HPP
#define ASLAM_CALCULATE_OVERLAPPING_FOV_HPP

#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/ImageMask.hpp>
#include <sm/assert_macros.hpp>
#include <sm/kinematics/Transformation.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace aslam {

void calculateOverlappingFOV(const cameras::CameraGeometryBase & camera1,
                             const cameras::CameraGeometryBase & camera2,
                             const sm::kinematics::Transformation& T_cam1_cam2,
                             cameras::ImageMask& outMask1,
                             cameras::ImageMask& outMask2,
                             double& outOverlappingRatio1,
                             double& outOverlappingRatio2,
                             const Eigen::VectorXi& sampleDistances,
                             double scale);

/// \brief Calculates the overlapping FoV of one camera. This is obviously zero.
/// \param[in] scale Allows to scale the resulting mask. By default no scaling (=1.0).
template<typename CAMERA_1_T>
void calculateOverlappingFOV(boost::shared_ptr<CAMERA_1_T> camera1,
                             cameras::ImageMask& outMask1,
                             double& outOverlappingRatio1,
                             const Eigen::VectorXi& sampleDistances,
                             double scale = 1.0);

/// \brief Calculates the overlapping FoV of one camera in the other one and the other way around.
/// \param[in] scale Allows to scale the resulting mask. By default no scaling (=1.0).
template<typename CAMERA_1_T, typename CAMERA_2_T>
void calculateOverlappingFOV(boost::shared_ptr<CAMERA_1_T> camera1,
                             boost::shared_ptr<CAMERA_1_T> camera2,
                             const sm::kinematics::Transformation& T_cam1_cam2,
                             cameras::ImageMask& outMask1,
                             cameras::ImageMask& outMask2,
                             double& outOverlappingRatio1,
                             double& outOverlappingRatio2,
                             const Eigen::VectorXi& sampleDistances,
                             double scale = 1.0);

}  // namespace aslam

#include "implementation/calculateOverlappingFov.hpp"

#endif /* ASLAM_CALCULATE_OVERLAPPING_FOV_HPP */
