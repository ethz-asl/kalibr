namespace aslam {

namespace cameras {

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection()
    : _xi(0.0),
      _fu(0.0),
      _fv(0.0),
      _cu(0.0),
      _cv(0.0),
      _ru(1),
      _rv(1) {
  updateTemporaries();

}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection(const sm::PropertyTree & config)
    : _distortion(sm::PropertyTree(config, "distortion")) {
  _xi = config.getDouble("xi");
  _fu = config.getDouble("fu");
  _fv = config.getDouble("fv");
  _cu = config.getDouble("cu");
  _cv = config.getDouble("cv");
  _ru = config.getInt("ru");
  _rv = config.getInt("rv");

  updateTemporaries();
}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection(double xi, double focalLengthU,
                                             double focalLengthV,
                                             double imageCenterU,
                                             double imageCenterV,
                                             int resolutionU, int resolutionV,
                                             distortion_t distortion)
    : _xi(xi),
      _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV),
      _distortion(distortion) {
  // 0
  updateTemporaries();

}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::OmniProjection(double xi, double focalLengthU,
                                             double focalLengthV,
                                             double imageCenterU,
                                             double imageCenterV,
                                             int resolutionU, int resolutionV)
    : _xi(xi),
      _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV) {
  updateTemporaries();
}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T>::~OmniProjection() {
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);
  outKeypoint.derived().resize(2);
  //    SM_OUT(p.transpose());
  double d = p.norm();

  // Check if point will lead to a valid projection
  if (p[2] <= -(_fov_parameter * d))
    return false;

  double rz = 1.0 / (p[2] + _xi * d);
  outKeypoint[0] = p[0] * rz;
  outKeypoint[1] = p[1] * rz;
  //std::cout << "normalize\n";
  //SM_OUT(d);
  //SM_OUT(rz);
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  _distortion.distort(outKeypoint);
  //std::cout << "distort\n";
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;
  //std::cout << "project\n";
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  // Check if keypoint lies on the sensor
  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool OmniProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst,
    const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JP>, 2, 3);

  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);
  outKeypoint.derived().resize(2);

  // Jacobian:
  Eigen::MatrixBase<DERIVED_JP> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
  J.derived().resize(KeypointDimension, 3);
  J.setZero();

  double d = p.norm();

  // Check if point will lead to a valid projection
  if (p[2] <= -(_fov_parameter * d))
    return false;

  // project the point
  double rz = 1.0 / (p[2] + _xi * d);
  outKeypoint[0] = p[0] * rz;
  outKeypoint[1] = p[1] * rz;

  // Calculate jacobian
  rz = rz * rz / d;
  J(0, 0) = rz * (d * p[2] + _xi * (p[1] * p[1] + p[2] * p[2]));
  J(1, 0) = -rz * _xi * p[0] * p[1];
  J(0, 1) = J(1, 0);
  J(1, 1) = rz * (d * p[2] + _xi * (p[0] * p[0] + p[2] * p[2]));
  rz = rz * (-_xi * p[2] - d);
  J(0, 2) = p[0] * rz;
  J(1, 2) = p[1] * rz;

  Eigen::Matrix2d Jd;
  _distortion.distort(outKeypoint, Jd);

  rz = _fu * (J(0, 0) * Jd(0, 0) + J(1, 0) * Jd(0, 1));
  J(1, 0) = _fv * (J(0, 0) * Jd(1, 0) + J(1, 0) * Jd(1, 1));
  J(0, 0) = rz;

  rz = _fu * (J(0, 1) * Jd(0, 0) + J(1, 1) * Jd(0, 1));
  J(1, 1) = _fv * (J(0, 1) * Jd(1, 0) + J(1, 1) * Jd(1, 1));
  J(0, 1) = rz;

  rz = _fu * (J(0, 2) * Jd(0, 0) + J(1, 2) * Jd(0, 1));
  J(1, 2) = _fv * (J(0, 2) * Jd(1, 0) + J(1, 2) * Jd(1, 1));
  J(0, 2) = rz;

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;

  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & ph,
    const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  // hope this works... (required to have valid static asserts)
  if (ph[3] < 0)
    return euclideanToKeypoint(-ph.derived().template head<3>(), outKeypoint);
  else
    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool OmniProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & ph,
    const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JP>, 2, 4);

  Eigen::MatrixBase<DERIVED_JP> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
  J.derived().resize(KeypointDimension, 4);
  J.setZero();

  if (ph[3] < 0) {
    bool success = euclideanToKeypoint(
        -ph.derived().template head<3>(), outKeypoint,
        J.derived().template topLeftCorner<2, 3>());
    J = -J;
    return success;
  } else {
    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint,
                               J.derived().template topLeftCorner<2, 3>());
  }
}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  // Unproject...
  outPoint[0] = _recip_fu * (keypoint[0] - _cu);
  outPoint[1] = _recip_fv * (keypoint[1] - _cv);

  // Re-distort
  _distortion.undistort(outPoint.derived().template head<2>());

  double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];

  if (!isUndistortedKeypointValid(rho2_d))
    return false;

  outPoint[2] = 1
      - _xi * (rho2_d + 1) / (_xi + sqrt(1 + (1 - _xi * _xi) * rho2_d));

  return true;

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool OmniProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 3, 2);

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  // Unproject...
  outPoint[0] = _recip_fu * (keypoint[0] - _cu);
  outPoint[1] = _recip_fv * (keypoint[1] - _cv);

  // Re-distort
  Eigen::MatrixXd Jd(2, 2);
  _distortion.undistort(outPoint.derived().template head<2>(), Jd);

  double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];

  if (!isUndistortedKeypointValid(rho2_d))
    return false;

  double tmpZ = sqrt(-(rho2_d) * (_xi * _xi - 1.0) + 1.0);
  double tmpA = _xi + tmpZ;
  double recip_tmpA = 1.0 / tmpA;

  outPoint[2] = 1 - _xi * (rho2_d + 1) * recip_tmpA;

  Eigen::MatrixBase<DERIVED_JK> & mbJk =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  DERIVED_JK & Jk = mbJk.derived();

  double r0 = outPoint[0];
  double r1 = outPoint[1];

  double recip_tmpA2 = recip_tmpA * recip_tmpA;
  //double recip_tmpA2 = 1.0/tmpA2;
  double tmpB = 1.0 / tmpZ;
  Jk(0, 0) = Jd(0, 0) * _recip_fu;
  Jk(0, 1) = Jd(0, 1) * _recip_fv;
  Jk(1, 0) = Jd(1, 0) * _recip_fu;
  Jk(1, 1) = Jd(1, 1) * _recip_fv;

  double mul = -_xi
      * (2.0 * recip_tmpA
          + recip_tmpA2 * (_xi * _xi - 1.0) * tmpB * (rho2_d + 1.0));
  Eigen::Vector2d J3;
  J3[0] = r0 * mul;
  J3[1] = r1 * mul;

  Jk.row(2) = J3.transpose() * Jd
      * Eigen::Vector2d(_recip_fu, _recip_fv).asDiagonal();

  //ASLAM_CAMERAS_ESTIMATE_JACOBIAN(this,keypointToEuclidean, keypoint, 1e-5, Jk);

  return true;

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p.derived().resize(4);
  p[3] = 0.0;
  return keypointToEuclidean(keypoint, p.derived().template head<3>());

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool OmniProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 4, 2);

  Eigen::MatrixBase<DERIVED_JK> & Jk =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  Jk.derived().resize(4, 2);
  Jk.setZero();

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p.derived().resize(4);
  p[3] = 0.0;

  return keypointToEuclidean(keypoint, p.template head<3>(),
                             Jk.template topLeftCorner<3, 2>());

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void OmniProjection<DISTORTION_T>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JI>, (int) KeypointDimension, 5);

  Eigen::MatrixBase<DERIVED_JI> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi);
  J.derived().resize(KeypointDimension, 5);
  J.setZero();

  keypoint_t kp;
  double d = p.norm();
  double rz = 1.0 / (p[2] + _xi * d);
  kp[0] = p[0] * rz;
  kp[1] = p[1] * rz;

  Eigen::Vector2d Jxi;
  Jxi[0] = -kp[0] * d * rz;
  Jxi[1] = -kp[1] * d * rz;

  Eigen::Matrix2d Jd;
  _distortion.distort(kp, Jd);

  Jd.row(0) *= _fu;
  Jd.row(1) *= _fv;
  J.col(0) = Jd * Jxi;

  J(0, 1) = kp[0];
  J(0, 3) = 1;

  J(1, 2) = kp[1];
  J(1, 4) = 1;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void OmniProjection<DISTORTION_T>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);

  double d = p.norm();
  double rz = 1.0 / (p[2] + _xi * d);
  keypoint_t kp;
  kp[0] = p[0] * rz;
  kp[1] = p[1] * rz;

  _distortion.distortParameterJacobian(kp, outJd);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);

  J.row(0) *= _fu;
  J.row(1) *= _fv;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void OmniProjection<DISTORTION_T>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);

  if (p[3] < 0.0) {
    euclideanToKeypointIntrinsicsJacobian(-p.derived().template head<3>(),
                                          outJi);
  } else {
    euclideanToKeypointIntrinsicsJacobian(p.derived().template head<3>(),
                                          outJi);
  }

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void OmniProjection<DISTORTION_T>::homogeneousToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);

  if (p[3] < 0.0) {
    euclideanToKeypointDistortionJacobian(-p.derived().template head<3>(),
                                          outJd);
  } else {
    euclideanToKeypointDistortionJacobian(p.derived().template head<3>(),
                                          outJd);
  }

}

template<typename DISTORTION_T>
template<class Archive>
void OmniProjection<DISTORTION_T>::load(Archive & ar,
                                        const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_xi);
  ar >> BOOST_SERIALIZATION_NVP(_fu);
  ar >> BOOST_SERIALIZATION_NVP(_fv);
  ar >> BOOST_SERIALIZATION_NVP(_cu);
  ar >> BOOST_SERIALIZATION_NVP(_cv);
  ar >> BOOST_SERIALIZATION_NVP(_ru);
  ar >> BOOST_SERIALIZATION_NVP(_rv);
  ar >> BOOST_SERIALIZATION_NVP(_distortion);

  updateTemporaries();
}

template<typename DISTORTION_T>
template<class Archive>
void OmniProjection<DISTORTION_T>::save(Archive & ar,
                                        const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_xi);
  ar << BOOST_SERIALIZATION_NVP(_fu);
  ar << BOOST_SERIALIZATION_NVP(_fv);
  ar << BOOST_SERIALIZATION_NVP(_cu);
  ar << BOOST_SERIALIZATION_NVP(_cv);
  ar << BOOST_SERIALIZATION_NVP(_ru);
  ar << BOOST_SERIALIZATION_NVP(_rv);
  ar << BOOST_SERIALIZATION_NVP(_distortion);

}

// \brief creates a random valid keypoint.
template<typename DISTORTION_T>
Eigen::VectorXd OmniProjection<DISTORTION_T>::createRandomKeypoint() const {

  // This is tricky...The camera model defines a circle on the normalized image
  // plane and the projection equations don't work outside of it.
  // With some manipulation, we can see that, on the normalized image plane,
  // the edge of this circle is at u^2 + v^2 = 1/(xi^2 - 1)
  // So: this function creates keypoints inside this boundary.

  // Create a point on the normalized image plane inside the boundary.
  // This is not efficient, but it should be correct.

  Eigen::Vector2d u(_ru + 1, _rv + 1);

  while (u[0] <= 0 || u[0] >= _ru - 1 || u[1] <= 0 || u[1] >= _rv - 1) {
    u.setRandom();
    u = u - Eigen::Vector2d(0.5, 0.5);
    u /= u.norm();
    u *= ((double) rand() / (double) RAND_MAX) * _one_over_xixi_m_1;

    // Now we run the point through distortion and projection.
    // Apply distortion
    _distortion.distort(u);

    u[0] = _fu * u[0] + _cu;
    u[1] = _fv * u[1] + _cv;
  }

  return u;
}

// \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
template<typename DISTORTION_T>
Eigen::Vector3d OmniProjection<DISTORTION_T>::createRandomVisiblePoint(
    double depth) const {
  Eigen::VectorXd y = createRandomKeypoint();
  Eigen::Vector3d p;
  keypointToEuclidean(y, p);

  if (depth < 0.0) {
    depth = ((double) rand() / (double) RAND_MAX) * 100.0;
  }

  p /= p.norm();

  // Muck with the depth. This doesn't change the pointing direction.
  p *= depth;
  return p;

}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  return keypoint(0) >= 0 && keypoint(0) < ru() && keypoint(1) >= 0
      && keypoint(1) < rv();
}

template<typename DISTORTION_T>
bool OmniProjection<DISTORTION_T>::isUndistortedKeypointValid(
    const double rho2_d) const {
  return xi() <= 1.0 || rho2_d <= _one_over_xixi_m_1;
}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool OmniProjection<DISTORTION_T>::isLiftable(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  // Unproject...
  Eigen::Vector2d y;
  y[0] = _recip_fu * (keypoint[0] - _cu);
  y[1] = _recip_fv * (keypoint[1] - _cv);

  // Re-distort
  _distortion.undistort(y);

  // Now check if it is on the sensor
  double rho2_d = y[0] * y[0] + y[1] * y[1];
  return isUndistortedKeypointValid(rho2_d);
}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> & p) const {
  keypoint_t k;
  return euclideanToKeypoint(p, k);

}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool OmniProjection<DISTORTION_T>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> & ph) const {
  keypoint_t k;
  return homogeneousToKeypoint(ph, k);
}

template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::updateTemporaries() {
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;
  _one_over_xixi_m_1 = 1.0 / (_xi * _xi - 1);
  _fov_parameter = (_xi <= 1.0) ? _xi : 1 / _xi;
}

// aslam::backend compatibility
template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::update(const double * v) {
  _xi += v[0];
  _fu += v[1];
  _fv += v[2];
  _cu += v[3];
  _cv += v[4];

  updateTemporaries();

}
template<typename DISTORTION_T>
int OmniProjection<DISTORTION_T>::minimalDimensions() const {
  return 5;
}

template<typename DISTORTION_T>
Eigen::Vector2i OmniProjection<DISTORTION_T>::parameterSize() const {
  return Eigen::Vector2i(5, 1);
}

template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::getParameters(Eigen::MatrixXd & P) const {
  P.resize(5, 1);
  P << _xi, _fu, _fv, _cu, _cv;
}
template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::setParameters(const Eigen::MatrixXd & P) {
  SM_ASSERT_EQ(std::runtime_error, P.rows(), 5, "Incorrect size");
  SM_ASSERT_EQ(std::runtime_error, P.cols(), 1, "Incorrect size");
  _xi = P(0, 0);
  _fu = P(1, 0);
  _fv = P(2, 0);
  _cu = P(3, 0);
  _cv = P(4, 0);

  updateTemporaries();
}

template<typename DISTORTION_T>
bool OmniProjection<DISTORTION_T>::isBinaryEqual(
    const OmniProjection<distortion_t> & rhs) const {
  return _xi == rhs._xi && _fu == rhs._fu && _fv == rhs._fv && _cu == rhs._cu
      && _cv == rhs._cv && _ru == rhs._ru && _rv == rhs._rv
      && _recip_fu == rhs._recip_fu && _recip_fv == rhs._recip_fv
      && _fu_over_fv == rhs._fu_over_fv
      && _one_over_xixi_m_1 == rhs._one_over_xixi_m_1
      && _distortion.isBinaryEqual(rhs._distortion);
}

template<typename DISTORTION_T>
OmniProjection<DISTORTION_T> OmniProjection<DISTORTION_T>::getTestProjection() {
  return OmniProjection<DISTORTION_T>(0.9, 400, 400, 320, 240, 640, 480,
                                      DISTORTION_T::getTestDistortion());
}

template<typename DISTORTION_T>
void OmniProjection<DISTORTION_T>::resizeIntrinsics(double scale) {
  _fu *= scale;
  _fv *= scale;
  _cu *= scale;
  _cv *= scale;
  _ru = _ru * scale;
  _rv = _rv * scale;

  updateTemporaries();
}

namespace detail {

inline double square(double x) {
  return x * x;
}
inline float square(float x) {
  return x * x;
}
inline double hypot(double a, double b) {
  return sqrt(square(a) + square(b));
}

}  // namespace detail

/// \brief initialize the intrinsics based on one view of a gridded calibration target
/// \return true on success
///
/// These functions were developed with the help of Lionel Heng and the excellent camodocal
/// https://github.com/hengli/camodocal
template<typename DISTORTION_T>
bool OmniProjection<DISTORTION_T>::initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations) {
  using detail::square;
  using detail::hypot;

  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
  SM_ASSERT_TRUE(Exception, observations.size() != 0, "Need min. one observation");


  // First, initialize the image center at the center of the image.
  _xi = 1.0;
  _cu = (observations[0].imCols() - 1.0) / 2.0;
  _cv = (observations[0].imRows() - 1.0) / 2.0;
  _ru = observations[0].imCols();
  _rv = observations[0].imRows();
  _distortion.clear();

  /// Initialize some temporaries needed.
  double gamma0 = 0.0;
  double minReprojErr = std::numeric_limits<double>::max();

  // Now we try to find a non-radial line to initialize the focal length
  bool success = false;

  //go though all images and pick the best estimate (=lowest mean reproj. err)
  for (size_t i = 0; i < observations.size(); ++i) {
    const GridCalibrationTargetObservation& obs = observations.at(i);
    SM_ASSERT_TRUE(Exception, obs.target(), "The GridCalibrationTargetObservation has no target object");
    const GridCalibrationTargetBase & target = *obs.target();

    //check all corner rows of the target
    for (size_t r = 0; r < target.rows(); ++r)
    {
      // Grab all the valid corner points for this observation
      cv::Mat P(target.cols(), 4, CV_64F);
      size_t count = 0;

      for (size_t c = 0; c < target.cols(); ++c) {
        Eigen::Vector2d imagePoint;
        Eigen::Vector3d gridPoint;

        if (obs.imageGridPoint(r, c, imagePoint)) {
          double u = imagePoint[0] - _cu;
          double v = imagePoint[1] - _cv;
          P.at<double>(count, 0) = u;
          P.at<double>(count, 1) = v;
          P.at<double>(count, 2) = 0.5;
          P.at<double>(count, 3) = -0.5 * (square(u) + square(v));
          ++count;
        }
      }

      // MIN_CORNERS is an arbitrary threshold for the number of corners
      const size_t MIN_CORNERS = 4;
      if (count > MIN_CORNERS)
      {
        // Resize P to fit with the count of valid points.
        cv::Mat C;
        cv::SVD::solveZ(P.rowRange(0, count), C);

        double t = square(C.at<double>(0)) + square(C.at<double>(1)) + C.at<double>(2) * C.at<double>(3);
        if (t < 0) {
          //SM_DEBUG_STREAM("Skipping a bad SVD solution on row " << r);
          continue;
        }

        // check that line image is not radial
        double d = sqrt(1.0 / t);
        double nx = C.at<double>(0) * d;
        double ny = C.at<double>(1) * d;
        if (hypot(nx, ny) > 0.95) {
          //SM_DEBUG_STREAM("Skipping a radial line on row " << r);
          continue;
        }

        //calculate the focal length estimate
        double nz = sqrt(1.0 - square(nx) - square(ny));
        double gamma = fabs(C.at<double>(2) * d / nz);

        //calculate the reprojection error using this estimate
        //SM_DEBUG_STREAM("Testing a focal length estimate of " << gamma);
        _fu = gamma;
        _fv = gamma;
        updateTemporaries();

        sm::kinematics::Transformation T_target_camera;
        if (!estimateTransformation(obs, T_target_camera)) {
          //SM_DEBUG_STREAM("Skipping row " << r << " as the transformation estimation failed.");
          continue;
        }

        double reprojErr = 0.0;
        size_t numReprojected = computeReprojectionError(obs, T_target_camera, reprojErr);
        if (numReprojected > MIN_CORNERS)
        {
          double avgReprojErr = reprojErr / numReprojected;

          if (avgReprojErr < minReprojErr)
          {
            //SM_DEBUG_STREAM("Row " << r << " produced the new best estimate: " << avgReprojErr << " < " << minReprojErr);
            minReprojErr = avgReprojErr;
            gamma0 = gamma;
            success = true;
          }
        }
      }  // If this observation has enough valid corners
      else {
        //SM_DEBUG_STREAM("Skipping row " << r << " because it only had " << count << " corners. Minimum: " << MIN_CORNERS);
      }
    }  // For each row in the image.
  } //For each image

  //set the parameters
  _fu = gamma0;
  _fv = gamma0;
  updateTemporaries();
  return success;
}  // initializeIntrinsics()

template<typename DISTORTION_T>
size_t OmniProjection<DISTORTION_T>::computeReprojectionError(
    const GridCalibrationTargetObservation & obs,
    const sm::kinematics::Transformation & T_target_camera,
    double & outErr) const {
  outErr = 0.0;
  size_t count = 0;
  sm::kinematics::Transformation T_camera_target = T_target_camera.inverse();

  for (size_t i = 0; i < obs.target()->size(); ++i) {
    Eigen::Vector2d y, yhat;
    if (obs.imagePoint(i, y)
        && euclideanToKeypoint(T_camera_target * obs.target()->point(i), yhat)) {
      outErr += (y - yhat).norm();
      ++count;
    }
  }

  return count;
}

/// \brief estimate the transformation of the camera with respect to the calibration target
///        On success out_T_t_c is filled in with the transformation that takes points from
///        the camera frame to the target frame
/// \return true on success
///
/// These functions were developed with the help of Lionel Heng and the excellent camodocal
/// https://github.com/hengli/camodocal
template<typename DISTORTION_T>
bool OmniProjection<DISTORTION_T>::estimateTransformation(
    const GridCalibrationTargetObservation & obs,
    sm::kinematics::Transformation & out_T_t_c) const {
  using detail::square;
  std::vector<cv::Point2f> Ms;
  std::vector<cv::Point3f> Ps;

  // Get the observed corners in the image and target frame
  obs.getCornersImageFrame(Ms);
  obs.getCornersTargetFrame(Ps);

  // Convert all target corners to a fakey pinhole view.
  size_t count = 0;
  for (size_t i = 0; i < Ms.size(); ++i) {
    Eigen::Vector3d targetPoint(Ps[i].x, Ps[i].y, Ps[i].z);
    Eigen::Vector2d imagePoint(Ms[i].x, Ms[i].y);
    Eigen::Vector3d backProjection;

    if (keypointToEuclidean(imagePoint, backProjection)
        && backProjection.normalized()[2] > std::cos(80.0*M_PI/180.0)) {
      double x = backProjection[0];
      double y = backProjection[1];
      double z = backProjection[2];
      Ps.at(count).x = targetPoint[0];
      Ps.at(count).y = targetPoint[1];
      Ps.at(count).z = targetPoint[2];

      Ms.at(count).x = x / z;
      Ms.at(count).y = y / z;
      ++count;
    } else {
//      SM_DEBUG_STREAM(
//          "Skipping point " << i << ", point was observed: " << imagePoint
//              << ", projection success: "
//              << keypointToEuclidean(imagePoint, backProjection)
//              << ", in front of camera: " << (backProjection[2] > 0.0)
//              << "image point: " << imagePoint.transpose()
//              << ", backProjection: " << backProjection.transpose()
//              << ", camera params (xi,fu,fv,cu,cv):" << xi() << ", " << fu()
//              << ", " << fv() << ", " << cu() << ", " << cv());
    }
  }

  Ps.resize(count);
  Ms.resize(count);

  std::vector<double> distCoeffs(4, 0.0);

  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);

  if (Ps.size() < 4) {
//    SM_DEBUG_STREAM(
//        "At least 4 points are needed for calling PnP. Found " << Ps.size());
    return false;
  }

  // Call the OpenCV pnp function.
//  SM_DEBUG_STREAM("Calling solvePnP with " << Ps.size() << " world points and "
//                  << Ms.size() << " image points");
  cv::solvePnP(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);

  // convert the rvec/tvec to a transformation
  cv::Mat C_camera_model = cv::Mat::eye(3, 3, CV_64F);
  Eigen::Matrix4d T_camera_model = Eigen::Matrix4d::Identity();
  cv::Rodrigues(rvec, C_camera_model);
  for (int r = 0; r < 3; ++r) {
    T_camera_model(r, 3) = tvec.at<double>(r, 0);
    for (int c = 0; c < 3; ++c) {
      T_camera_model(r, c) = C_camera_model.at<double>(r, c);
    }
  }

  out_T_t_c.set(T_camera_model.inverse());
  return true;
}

}  // namespace cameras
}  // namespace aslam
