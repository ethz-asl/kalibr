namespace aslam {
namespace cameras {

template<typename DERIVED_Y>
void EquidistantDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  double r, theta, theta2, theta4, theta6, theta8, thetad, scaling;

  r = sqrt(y[0] * y[0] + y[1] * y[1]);
  theta = atan(r);
  theta2 = theta * theta;
  theta4 = theta2 * theta2;
  theta6 = theta4 * theta2;
  theta8 = theta4 * theta4;
  thetad = theta
      * (1 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);

  scaling = (r > 1e-8) ? thetad / r : 1.0;
  y[0] *= scaling;
  y[1] *= scaling;
}

template<typename DERIVED_Y, typename DERIVED_JY>
void EquidistantDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  //double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  Eigen::MatrixBase<DERIVED_JY> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJy);
  J.derived().resize(2, 2);
  J.setZero();

  Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
  y.derived().resize(2);

  double r, theta, theta2, theta4, theta6, theta8, thetad, scaling;

  //MATLAB generated Jacobian
  J(0, 0) = atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1])
      * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
          + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
          + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
          + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0)
      + y[0] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
          / sqrt(y[0] * y[0] + y[1] * y[1])
          * ((_k2 * y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 3.0) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 4.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0)
              + (_k3 * y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 5.0)
                  * 1.0 / sqrt(y[0] * y[0] + y[1] * y[1]) * 6.0)
                  / (y[0] * y[0] + y[1] * y[1] + 1.0)
              + (_k4 * y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 7.0)
                  * 1.0 / sqrt(y[0] * y[0] + y[1] * y[1]) * 8.0)
                  / (y[0] * y[0] + y[1] * y[1] + 1.0)
              + (_k1 * y[0] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
                  / sqrt(y[0] * y[0] + y[1] * y[1]) * 2.0)
                  / (y[0] * y[0] + y[1] * y[1] + 1.0))
      + ((y[0] * y[0])
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0))
          / ((y[0] * y[0] + y[1] * y[1]) * (y[0] * y[0] + y[1] * y[1] + 1.0))
      - (y[0] * y[0]) * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
          / pow(y[0] * y[0] + y[1] * y[1], 3.0 / 2.0)
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0);
  J(0, 1) = y[0] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1])
      * ((_k2 * y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 3.0) * 1.0
          / sqrt(y[0] * y[0] + y[1] * y[1]) * 4.0)
          / (y[0] * y[0] + y[1] * y[1] + 1.0)
          + (_k3 * y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 5.0) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 6.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0)
          + (_k4 * y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 7.0) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 8.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0)
          + (_k1 * y[1] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 2.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0))
      + (y[0] * y[1]
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0))
          / ((y[0] * y[0] + y[1] * y[1]) * (y[0] * y[0] + y[1] * y[1] + 1.0))
      - y[0] * y[1] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
          / pow(y[0] * y[0] + y[1] * y[1], 3.0 / 2.0)
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0);
  J(1, 0) = y[1] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1])
      * ((_k2 * y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 3.0) * 1.0
          / sqrt(y[0] * y[0] + y[1] * y[1]) * 4.0)
          / (y[0] * y[0] + y[1] * y[1] + 1.0)
          + (_k3 * y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 5.0) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 6.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0)
          + (_k4 * y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 7.0) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 8.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0)
          + (_k1 * y[0] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 2.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0))
      + (y[0] * y[1]
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0))
          / ((y[0] * y[0] + y[1] * y[1]) * (y[0] * y[0] + y[1] * y[1] + 1.0))
      - y[0] * y[1] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
          / pow(y[0] * y[0] + y[1] * y[1], 3.0 / 2.0)
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0);
  J(1, 1) = atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1])
      * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
          + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
          + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
          + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0)
      + y[1] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
          / sqrt(y[0] * y[0] + y[1] * y[1])
          * ((_k2 * y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 3.0) * 1.0
              / sqrt(y[0] * y[0] + y[1] * y[1]) * 4.0)
              / (y[0] * y[0] + y[1] * y[1] + 1.0)
              + (_k3 * y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 5.0)
                  * 1.0 / sqrt(y[0] * y[0] + y[1] * y[1]) * 6.0)
                  / (y[0] * y[0] + y[1] * y[1] + 1.0)
              + (_k4 * y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 7.0)
                  * 1.0 / sqrt(y[0] * y[0] + y[1] * y[1]) * 8.0)
                  / (y[0] * y[0] + y[1] * y[1] + 1.0)
              + (_k1 * y[1] * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
                  / sqrt(y[0] * y[0] + y[1] * y[1]) * 2.0)
                  / (y[0] * y[0] + y[1] * y[1] + 1.0))
      + ((y[1] * y[1])
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0))
          / ((y[0] * y[0] + y[1] * y[1]) * (y[0] * y[0] + y[1] * y[1] + 1.0))
      - (y[1] * y[1]) * atan(sqrt(y[0] * y[0] + y[1] * y[1])) * 1.0
          / pow(y[0] * y[0] + y[1] * y[1], 3.0 / 2.0)
          * (_k1 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 2.0)
              + _k2 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 4.0)
              + _k3 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 6.0)
              + _k4 * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 8.0) + 1.0);

  r = sqrt(y[0] * y[0] + y[1] * y[1]);
  theta = atan(r);
  theta2 = theta * theta;
  theta4 = theta2 * theta2;
  theta6 = theta4 * theta2;
  theta8 = theta4 * theta4;
  thetad = theta
      * (1 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);

  scaling = (r > 1e-8) ? thetad / r : 1.0;
  y[0] *= scaling;
  y[1] *= scaling;
}

template<typename DERIVED>
void EquidistantDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & yconst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);

  Eigen::MatrixBase<DERIVED> & y =
      const_cast<Eigen::MatrixBase<DERIVED> &>(yconst);
  y.derived().resize(2);

  double theta, theta2, theta4, theta6, theta8, thetad, scaling;

  thetad = sqrt(y[0] * y[0] + y[1] * y[1]);
  theta = thetad;  // initial guess
  for (int i = 20; i > 0; i--) {
    theta2 = theta * theta;
    theta4 = theta2 * theta2;
    theta6 = theta4 * theta2;
    theta8 = theta4 * theta4;
    theta = thetad
        / (1 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);
  }
  scaling = tan(theta) / thetad;

  y[0] *= scaling;
  y[1] *= scaling;
}

template<typename DERIVED, typename DERIVED_JY>
void EquidistantDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);

  Eigen::MatrixBase<DERIVED> & y =
      const_cast<Eigen::MatrixBase<DERIVED> &>(yconst);
  y.derived().resize(2);

  // we use f^-1 ' = ( f'(f^-1) ) '
  // with f^-1 the undistortion
  // and  f the distortion
  undistort(y);  // first get the undistorted image

  Eigen::Vector2d kp = y;
  Eigen::Matrix2d Jd;
  distort(kp, Jd);

  // now y = f^-1(y0)
  DERIVED_JY & J = const_cast<DERIVED_JY &>(outJy.derived());

  J = Jd.inverse();

}

template<typename DERIVED_Y, typename DERIVED_JD>
void EquidistantDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y> & y,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JD>, 2, 4);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
  J.derived().resize(2, 4);
  J.setZero();

  J(0, 0) = y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 3.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(0, 1) = y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 5.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(0, 2) = y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 7.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(0, 3) = y[0] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 9.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(1, 0) = y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 3.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(1, 1) = y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 5.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(1, 2) = y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 7.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);
  J(1, 3) = y[1] * pow(atan(sqrt(y[0] * y[0] + y[1] * y[1])), 9.0) * 1.0
      / sqrt(y[0] * y[0] + y[1] * y[1]);

  /*
   double y0 = imageY[0];
   double y1 = imageY[1];
   double r2 = y0*y0 + y1*y1;
   double r4 = r2*r2;

   Eigen::MatrixBase<DERIVED_JD> & J = const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
   J.derived().resize(2,4);
   J.setZero();

   J(0,0) = y0*r2;
   J(0,1) = y0*r4;
   J(0,2) = 2.0*y0*y1;
   J(0,3) = r2 + 2.0*y0*y0;

   J(1,0) = y1*r2;
   J(1,1) = y1*r4;
   J(1,2) = r2 + 2.0*y1*y1;
   J(1,3) = 2.0*y0*y1;
   */
}

template<class Archive>
void EquidistantDistortion::load(Archive & ar, const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_k1);
  ar >> BOOST_SERIALIZATION_NVP(_k2);
  ar >> BOOST_SERIALIZATION_NVP(_k3);
  ar >> BOOST_SERIALIZATION_NVP(_k4);
}

template<class Archive>
void EquidistantDistortion::save(Archive & ar,
                                 const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_k1);
  ar << BOOST_SERIALIZATION_NVP(_k2);
  ar << BOOST_SERIALIZATION_NVP(_k3);
  ar << BOOST_SERIALIZATION_NVP(_k4);
}

}  // namespace cameras
}  // namespace aslam
