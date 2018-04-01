namespace aslam {

namespace cameras {

template<typename DISTORTION_T>
DoubleSphereProjection<DISTORTION_T>::DoubleSphereProjection()
    : _xi1(0.0),
      _xi2(0.0),
      _fu(0.0),
      _fv(0.0),
      _cu(0.0),
      _cv(0.0),
      _ru(1),
      _rv(1) {
  updateTemporaries();

}

template<typename DISTORTION_T>
DoubleSphereProjection<DISTORTION_T>::DoubleSphereProjection(const sm::PropertyTree & config)
    : _distortion(sm::PropertyTree(config, "distortion")) {
  _xi1 = config.getDouble("xi1");
  _xi2 = config.getDouble("xi2");
  _fu = config.getDouble("fu");
  _fv = config.getDouble("fv");
  _cu = config.getDouble("cu");
  _cv = config.getDouble("cv");
  _ru = config.getInt("ru");
  _rv = config.getInt("rv");

  updateTemporaries();
}

template<typename DISTORTION_T>
DoubleSphereProjection<DISTORTION_T>::DoubleSphereProjection(double xi1, double xi2, double focalLengthU,
                                             double focalLengthV,
                                             double imageCenterU,
                                             double imageCenterV,
                                             int resolutionU, int resolutionV,
                                             distortion_t distortion)
    : _xi1(xi1),
      _xi2(xi2),
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
DoubleSphereProjection<DISTORTION_T>::DoubleSphereProjection(double xi1, double xi2, double focalLengthU,
                                             double focalLengthV,
                                             double imageCenterU,
                                             double imageCenterV,
                                             int resolutionU, int resolutionV)
    : _xi1(xi1),
      _xi2(xi2),
      _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV) {
  updateTemporaries();
}

template<typename DISTORTION_T>
DoubleSphereProjection<DISTORTION_T>::~DoubleSphereProjection() {
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool DoubleSphereProjection<DISTORTION_T>::euclideanToKeypoint(
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

  // Check if point will lead to a valid projection
  //if (p[2] <= -(_fov_parameter * d))
  //  return false;

  const double& x = p[0];
  const double& y = p[1];
  const double& z = p[2];

  double xx = x * x;
  double yy = y * y;
  double zz = z * z;

  double r2 = xx + yy;

  double d1_2 = r2 + zz;
  double d1 = std::sqrt(d1_2);

  double k = _xi1 * d1 + z;
  double kk = k * k;

  double d2_2 = r2 + kk;
  double d2 = std::sqrt(d2_2);

  double norm = _xi2 * d2 + (1 - _xi2) * k;
  double norm_inv = 1.0 / norm;

  outKeypoint[0] = p[0] * norm_inv;
  outKeypoint[1] = p[1] * norm_inv;
  //std::cout << "normalize\n";
  //SM_OUT(d);
  //SM_OUT(rz);
  //SM_OUT(outKeypoint[0]);
  //SM_OUT(outKeypoint[1]);

  //_distortion.distort(outKeypoint);
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
bool DoubleSphereProjection<DISTORTION_T>::euclideanToKeypoint(
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

  // Check if point will lead to a valid projection
  //if (p[2] <= -(_fov_parameter * d))
  //  return false;

  // project the point

  const double& x = p[0];
  const double& y = p[1];
  const double& z = p[2];

  double xx = x * x;
  double yy = y * y;
  double zz = z * z;

  double r2 = xx + yy;

  double d1_2 = r2 + zz;
  double d1 = std::sqrt(d1_2);
  double d1_inv = 1.0 / d1;

  double k = _xi1 * d1 + z;
  double kk = k * k;

  double d2_2 = r2 + kk;
  double d2 = std::sqrt(d2_2);
  double d2_inv = 1.0 / d2;

  double norm = _xi2 * d2 + (1 - _xi2) * k;
  double norm_inv = 1.0 / norm;
  double norm_inv2 = norm_inv*norm_inv;

  outKeypoint[0] = p[0] * norm_inv;
  outKeypoint[1] = p[1] * norm_inv;

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;

  double xy = x * y;
  double tt2 = _xi1 * z * d1_inv + 1;

  double d_norm_d_r2 =
        (_xi1 * (1 - _xi2) * d1_inv + _xi2 * (_xi1 * k * d1_inv + 1) * d2_inv) *
        norm_inv2;

  double tmp2 = ((1 - _xi2) * tt2 + _xi2 * k * tt2 * d2_inv) * norm_inv2;

  J(0, 0) = _fu * (norm_inv - xx * d_norm_d_r2);
  J(1, 0) = -_fv * xy * d_norm_d_r2;

  J(0, 1) = -_fu * xy * d_norm_d_r2;
  J(1, 1) = _fv * (norm_inv - yy * d_norm_d_r2);

  J(0, 2) = -_fu * x * tmp2;
  J(1, 2) = -_fv * y * tmp2;


  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool DoubleSphereProjection<DISTORTION_T>::homogeneousToKeypoint(
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
bool DoubleSphereProjection<DISTORTION_T>::homogeneousToKeypoint(
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
bool DoubleSphereProjection<DISTORTION_T>::keypointToEuclidean(
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
  //_distortion.undistort(outPoint.derived().template head<2>());

  double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];

  if (!isUndistortedKeypointValid(rho2_d))
    return false;

  outPoint[2] = 1
      - _xi2 * (rho2_d + 1) / (_xi2 + sqrt(1 + (1 - _xi2 * _xi2) * rho2_d));

  return true;

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool DoubleSphereProjection<DISTORTION_T>::keypointToEuclidean(
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

  double tmpZ = sqrt(-(rho2_d) * (_xi2 * _xi2 - 1.0) + 1.0);
  double tmpA = _xi2 + tmpZ;
  double recip_tmpA = 1.0 / tmpA;

  outPoint[2] = 1 - _xi2 * (rho2_d + 1) * recip_tmpA;

  // \todo analytical Jacobian
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

  double mul = -_xi2
      * (2.0 * recip_tmpA
          + recip_tmpA2 * (_xi2 * _xi2 - 1.0) * tmpB * (rho2_d + 1.0));
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
bool DoubleSphereProjection<DISTORTION_T>::keypointToHomogeneous(
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
bool DoubleSphereProjection<DISTORTION_T>::keypointToHomogeneous(
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
void DoubleSphereProjection<DISTORTION_T>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JI>, (int) KeypointDimension, 6);

  Eigen::MatrixBase<DERIVED_JI> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi);
  J.derived().resize(KeypointDimension, 5);
  J.setZero();

  keypoint_t kp;
  double d = p.norm();
  double rz = 1.0 / (p[2] + _xi2 * d);
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
void DoubleSphereProjection<DISTORTION_T>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);

  double d = p.norm();
  double rz = 1.0 / (p[2] + _xi2 * d);
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
void DoubleSphereProjection<DISTORTION_T>::homogeneousToKeypointIntrinsicsJacobian(
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
void DoubleSphereProjection<DISTORTION_T>::homogeneousToKeypointDistortionJacobian(
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
void DoubleSphereProjection<DISTORTION_T>::load(Archive & ar,
                                        const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_xi1);
  ar >> BOOST_SERIALIZATION_NVP(_xi2);
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
void DoubleSphereProjection<DISTORTION_T>::save(Archive & ar,
                                        const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_xi1);
  ar << BOOST_SERIALIZATION_NVP(_xi2);
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
Eigen::VectorXd DoubleSphereProjection<DISTORTION_T>::createRandomKeypoint() const {

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
Eigen::Vector3d DoubleSphereProjection<DISTORTION_T>::createRandomVisiblePoint(
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
bool DoubleSphereProjection<DISTORTION_T>::isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  return keypoint(0) >= 0 && keypoint(0) < ru() && keypoint(1) >= 0
      && keypoint(1) < rv();
}

template<typename DISTORTION_T>
bool DoubleSphereProjection<DISTORTION_T>::isUndistortedKeypointValid(
    const double rho2_d) const {
  return true; //xi() <= 1.0 || rho2_d <= _one_over_xixi_m_1;
}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool DoubleSphereProjection<DISTORTION_T>::isLiftable(
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
bool DoubleSphereProjection<DISTORTION_T>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> & p) const {
  keypoint_t k;
  return euclideanToKeypoint(p, k);

}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool DoubleSphereProjection<DISTORTION_T>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> & ph) const {
  keypoint_t k;
  return homogeneousToKeypoint(ph, k);
}

template<typename DISTORTION_T>
void DoubleSphereProjection<DISTORTION_T>::updateTemporaries() {
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;
  _one_over_xixi_m_1 = 1;// 1.0 / (_xi * _xi - 1);
  _fov_parameter = 1;//(_xi <= 1.0) ? _xi : 1 / _xi;
}

// aslam::backend compatibility
template<typename DISTORTION_T>
void DoubleSphereProjection<DISTORTION_T>::update(const double * v) {
  _xi1 += v[0];
  _xi2 += v[1];
  _fu += v[2];
  _fv += v[3];
  _cu += v[4];
  _cv += v[5];

  updateTemporaries();

}
template<typename DISTORTION_T>
int DoubleSphereProjection<DISTORTION_T>::minimalDimensions() const {
  return 5;
}

template<typename DISTORTION_T>
Eigen::Vector2i DoubleSphereProjection<DISTORTION_T>::parameterSize() const {
  return Eigen::Vector2i(5, 1);
}

template<typename DISTORTION_T>
void DoubleSphereProjection<DISTORTION_T>::getParameters(Eigen::MatrixXd & P) const {
  P.resize(6, 1);
  P << _xi1, _xi2, _fu, _fv, _cu, _cv;
}
template<typename DISTORTION_T>
void DoubleSphereProjection<DISTORTION_T>::setParameters(const Eigen::MatrixXd & P) {
  SM_ASSERT_EQ(std::runtime_error, P.rows(), 6, "Incorrect size");
  SM_ASSERT_EQ(std::runtime_error, P.cols(), 1, "Incorrect size");
  _xi1 = P(0, 0);
  _xi2 = P(1, 0);
  _fu = P(2, 0);
  _fv = P(3, 0);
  _cu = P(4, 0);
  _cv = P(5, 0);

  updateTemporaries();
}

template<typename DISTORTION_T>
bool DoubleSphereProjection<DISTORTION_T>::isBinaryEqual(
    const DoubleSphereProjection<distortion_t> & rhs) const {
  return _xi1 == rhs._xi1 && _xi2 == rhs._xi2 && _fu == rhs._fu && _fv == rhs._fv && _cu == rhs._cu
      && _cv == rhs._cv && _ru == rhs._ru && _rv == rhs._rv
      && _recip_fu == rhs._recip_fu && _recip_fv == rhs._recip_fv
      && _fu_over_fv == rhs._fu_over_fv
      && _one_over_xixi_m_1 == rhs._one_over_xixi_m_1
      && _distortion.isBinaryEqual(rhs._distortion);
}

template<typename DISTORTION_T>
DoubleSphereProjection<DISTORTION_T> DoubleSphereProjection<DISTORTION_T>::getTestProjection() {
  return DoubleSphereProjection<DISTORTION_T>(-0.11234234, 0.5234234, 200, 200, 320, 240, 640, 480,
                                      DISTORTION_T::getTestDistortion());
}

template<typename DISTORTION_T>
void DoubleSphereProjection<DISTORTION_T>::resizeIntrinsics(double scale) {
  _fu *= scale;
  _fv *= scale;
  _cu *= scale;
  _cv *= scale;
  _ru = _ru * scale;
  _rv = _rv * scale;

  updateTemporaries();
}

/// \brief initialize the intrinsics based on one view of a gridded calibration target
/// \return true on success
///
/// These functions were developed with the help of Lionel Heng and the excellent camodocal
/// https://github.com/hengli/camodocal
template<typename DISTORTION_T>
bool DoubleSphereProjection<DISTORTION_T>::initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations) {

  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
  SM_ASSERT_TRUE(Exception, observations.size() != 0, "Need min. one observation");

  bool success = true;

  //set the parameters
  // _fu = gamma0;
  // _fv = gamma0;
  updateTemporaries();
  return success;
}  // initializeIntrinsics()

template<typename DISTORTION_T>
size_t DoubleSphereProjection<DISTORTION_T>::computeReprojectionError(
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
bool DoubleSphereProjection<DISTORTION_T>::estimateTransformation(
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
        && backProjection[2] > 0.0) {
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
