namespace aslam {
namespace cameras {

template<typename DERIVED_Y>
void FovDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  Eigen::Matrix2d J;
  distort(y, J);
}

template<typename DERIVED_Y, typename DERIVED_JY>
void FovDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  Eigen::MatrixBase<DERIVED_JY> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJy);
  J.derived().resize(2, 2);
  J.setZero();

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  const double r_u = y.norm();
  const double r_u_cubed = r_u * r_u * r_u;
  const double tanwhalf = tan(_w / 2.);
  const double tanwhalfsq = tanwhalf * tanwhalf;
  const double atan_wrd = atan(2. * tanwhalf * r_u);
  double r_rd;

  if (_w * _w < 1e-5) {
    // Limit _w > 0.
    r_rd = 1.0;
  } else {
    if (r_u * r_u < 1e-5) {
      // Limit r_u > 0.
      r_rd = 2. * tanwhalf / _w;
    } else {
      r_rd = atan_wrd / (r_u * _w);
    }
  }

  const double& u = y(0);
  const double& v = y(1);

  if (_w * _w < 1e-5) {
    J.setIdentity();
  } else if (r_u * r_u < 1e-5) {
    J.setIdentity();
    // The coordinates get multiplied by an expression not depending on r_u.
    J *= (2. * tanwhalf / _w);
  } else {
    const double duf_du = (atan_wrd) / (_w * r_u)
              - (u * u * atan_wrd) / (_w * r_u_cubed)
              + (2 * u * u * tanwhalf)
              / (_w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));
    const double duf_dv = (2 * u * v * tanwhalf)
              / (_w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
              - (u * v * atan_wrd) / (_w * r_u_cubed);
    const double dvf_du = (2 * u * v * tanwhalf)
              / (_w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1))
              - (u * v * atan_wrd) / (_w * r_u_cubed);
    const double dvf_dv = (atan_wrd) / (_w * r_u)
              - (v * v * atan_wrd) / (_w * r_u_cubed)
              + (2 * v * v * tanwhalf)
              / (_w * (u * u + v * v) * (4 * tanwhalfsq * (u * u + v * v) + 1));

    J << duf_du, duf_dv,
         dvf_du, dvf_dv;
  }

  y *= r_rd;
}

template<typename DERIVED>
void FovDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & yconst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);

  Eigen::MatrixBase<DERIVED> & y =
      const_cast<Eigen::MatrixBase<DERIVED> &>(yconst);
  y.derived().resize(2);

  double mul2tanwby2 = tan(_w / 2.0) * 2.0;

  // Calculate distance from point to center.
  double r_d = y.norm();

  if (mul2tanwby2 == 0 || r_d == 0) {
    return;
  }

  // Calculate undistorted radius of point.
  double r_u;
  if (fabs(r_d * _w) <= kMaxValidAngle) {
    r_u = tan(r_d * _w) / (r_d * mul2tanwby2);
  } else {
    return;
  }

  y *= r_u;
}

template<typename DERIVED, typename DERIVED_JY>
void FovDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & /*yconst*/,
    const Eigen::MatrixBase<DERIVED_JY> & /*outJy*/) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  SM_ASSERT_TRUE(std::runtime_error, false, "Not implemented.");
}

template<typename DERIVED_Y, typename DERIVED_JD>
void FovDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y> & y,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JD>, 2, 4);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
  J.derived().resize(2, 1);
  J.setZero();

  const double tanwhalf = tan(_w / 2.);
  const double tanwhalfsq = tanwhalf * tanwhalf;
  const double r_u = y.norm();
  const double atan_wrd = atan(2. * tanwhalf * r_u);

  const double& u = y(0);
  const double& v = y(1);

  if (_w * _w < 1e-5) {
    J.setZero();
  }
  else if (r_u * r_u < 1e-5) {
    J.setOnes();
    J *= (_w - sin(_w)) / (_w * _w * cos(_w / 2) * cos(_w / 2));
  } else {
    const double dxd_d_w = (2 * u * (tanwhalfsq / 2 + 0.5))
          / (_w * (4 * tanwhalfsq * r_u * r_u + 1))
          - (u * atan_wrd) / (_w * _w * r_u);

    const double dyd_d_w = (2 * v * (tanwhalfsq / 2 + 0.5))
          / (_w * (4 * tanwhalfsq * r_u * r_u + 1))
          - (v * atan_wrd) / (_w * _w * r_u);

    J << dxd_d_w, dyd_d_w;
  }
}

template<class Archive>
void FovDistortion::load(Archive & ar, const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_w);
}

template<class Archive>
void FovDistortion::save(Archive & ar,
                                 const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_w);
}

}  // namespace cameras
}  // namespace aslam
