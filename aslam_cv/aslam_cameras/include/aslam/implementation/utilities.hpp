namespace aslam {

template<typename FRAME_T>
void doBackProjection(FRAME_T & frame) {
  typedef FRAME_T frame_t;
  typedef typename frame_t::keypoint_t keypoint_t;
  typedef typename frame_t::camera_geometry_t camera_geometry_t;

  const camera_geometry_t & geometry = frame.geometry();
  for (size_t k = 0; k < frame.numKeypoints(); ++k) {
    keypoint_t & kp = frame[k];
    Eigen::Vector3d bp(0.0, 0.0, 0.0);
    geometry.keypointToEuclidean(kp.y(), bp);
    bp.normalize();
    kp.setBackProjection(bp);
  }
}

template<typename FRAME_T>
void doBackProjectionWithUncertainty(FRAME_T & frame) {

  typedef FRAME_T frame_t;
  typedef typename frame_t::keypoint_t keypoint_t;
  typedef typename frame_t::camera_geometry_t camera_geometry_t;

  const camera_geometry_t & geometry = frame.geometry();
  for (size_t k = 0; k < frame.numKeypoints(); ++k) {
    keypoint_t & kp = frame[k];
    Eigen::Vector3d bp;
    typename camera_geometry_t::inverse_jacobian_t Jb;
    geometry.keypointToEuclidean(kp.y(), bp, Jb);

    Eigen::Matrix3d P = Jb * kp.invR().inverse() * Jb.transpose();
    sm::kinematics::UncertainVector3 b(bp, P);
    b.normalize();

    kp.setBackProjection(b);
  }

}

}  // namespace aslam
