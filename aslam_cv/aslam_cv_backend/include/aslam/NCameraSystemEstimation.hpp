#ifndef ASLAM_N_CAMERA_SYSTEM_ESTIMATION_HPP
#define ASLAM_N_CAMERA_SYSTEM_ESTIMATION_HPP

namespace aslam {

void createCameraSystemDesignVariables(NCameraSystem & cameraSystem,
                                       bool estimateIntrinsics,
                                       bool estimateExtrinsics);

}  // namespace aslam

#endif /* ASLAM_N_CAMERA_SYSTEM_ESTIMATION_HPP */
