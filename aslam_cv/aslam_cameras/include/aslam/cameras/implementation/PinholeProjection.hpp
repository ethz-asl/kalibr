#include <opencv2/core/eigen.hpp>
#include <Eigen/StdVector>

namespace aslam {

namespace cameras {

template<typename DISTORTION_T>
PinholeProjection<DISTORTION_T>::PinholeProjection()
    : _fu(0.0),
      _fv(0.0),
      _cu(0.0),
      _cv(0.0),
      _ru(0),
      _rv(0) {
  updateTemporaries();
}

template<typename DISTORTION_T>
PinholeProjection<DISTORTION_T>::PinholeProjection(
    const sm::PropertyTree & config)
    : _distortion(sm::PropertyTree(config, "distortion")) {
  _fu = config.getDouble("fu");
  _fv = config.getDouble("fv");
  _cu = config.getDouble("cu");
  _cv = config.getDouble("cv");
  _ru = config.getInt("ru");
  _rv = config.getInt("rv");

  updateTemporaries();
}

template<typename DISTORTION_T>
PinholeProjection<DISTORTION_T>::PinholeProjection(double focalLengthU,
                                                   double focalLengthV,
                                                   double imageCenterU,
                                                   double imageCenterV,
                                                   int resolutionU,
                                                   int resolutionV,
                                                   distortion_t distortion)
    : _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV),
      _distortion(distortion) {
  updateTemporaries();
}

template<typename DISTORTION_T>
PinholeProjection<DISTORTION_T>::PinholeProjection(double focalLengthU,
                                                   double focalLengthV,
                                                   double imageCenterU,
                                                   double imageCenterV,
                                                   int resolutionU,
                                                   int resolutionV)
    : _fu(focalLengthU),
      _fv(focalLengthV),
      _cu(imageCenterU),
      _cv(imageCenterV),
      _ru(resolutionU),
      _rv(resolutionV) {
  updateTemporaries();
}

template<typename DISTORTION_T>
PinholeProjection<DISTORTION_T>::~PinholeProjection() {
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool PinholeProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);

  outKeypoint.derived().resize(2);
  double rz = 1.0 / p[2];
  outKeypoint[0] = p[0] * rz;
  outKeypoint[1] = p[1] * rz;

  _distortion.distort(outKeypoint);

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;

  return isValid(outKeypoint) && p[2] > 0;
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool PinholeProjection<DISTORTION_T>::euclideanToKeypoint(
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

  double rz = 1.0 / p[2];
  double rz2 = rz * rz;

  outKeypoint[0] = p[0] * rz;
  outKeypoint[1] = p[1] * rz;

  Eigen::MatrixXd Jd;
  _distortion.distort(outKeypoint, Jd);  // distort and Jacobian wrt. keypoint

  // Jacobian including distortion
  J(0, 0) = _fu * Jd(0, 0) * rz;
  J(0, 1) = _fu * Jd(0, 1) * rz;
  J(0, 2) = -_fu * (p[0] * Jd(0, 0) + p[1] * Jd(0, 1)) * rz2;
  J(1, 0) = _fv * Jd(1, 0) * rz;
  J(1, 1) = _fv * Jd(1, 1) * rz;
  J(1, 2) = -_fv * (p[0] * Jd(1, 0) + p[1] * Jd(1, 1)) * rz2;

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;

  return isValid(outKeypoint) && p[2] > 0;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool PinholeProjection<DISTORTION_T>::homogeneousToKeypoint(
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
bool PinholeProjection<DISTORTION_T>::homogeneousToKeypoint(
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

  // hope this works... (required to have valid static asserts)
  return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint,
                             J.derived().template topLeftCorner<2, 3>());

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
bool PinholeProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  keypoint_t kp = keypoint;

  kp[0] = (kp[0] - _cu) / _fu;
  kp[1] = (kp[1] - _cv) / _fv;
  _distortion.undistort(kp);  // revert distortion

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  outPoint[0] = kp[0];
  outPoint[1] = kp[1];
  outPoint[2] = 1;

  return isValid(keypoint);

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool PinholeProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 3, 2);

  keypoint_t kp = keypoint;

  kp[0] = (kp[0] - _cu) / _fu;
  kp[1] = (kp[1] - _cv) / _fv;

  Eigen::MatrixXd Jd(2, 2);

  _distortion.undistort(kp, Jd);  // revert distortion

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  outPoint[0] = kp[0];
  outPoint[1] = kp[1];
  outPoint[2] = 1;

  Eigen::MatrixBase<DERIVED_JK> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  J.derived().resize(3, KeypointDimension);
  J.setZero();

  //J.derived()(0,0) = 1.0;
  //J.derived()(1,1) = _fu_over_fv;

  J.derived()(0, 0) = _recip_fu;
  J.derived()(1, 1) = _recip_fv;

  J *= Jd;

  return isValid(keypoint);

}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool PinholeProjection<DISTORTION_T>::keypointToHomogeneous(
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
bool PinholeProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 4, 2);

  Eigen::MatrixBase<DERIVED_JK> & Jk = const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);

  Jk.derived().resize(4, 2);
  Jk.setZero();

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p[3] = 0.0;

  return keypointToEuclidean(keypoint, p.template head<3>(),
                             Jk.template topLeftCorner<3, 2>());

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void PinholeProjection<DISTORTION_T>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JI>, 2, 4);

  Eigen::MatrixBase<DERIVED_JI> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi);
  J.derived().resize(KeypointDimension, 4);
  J.setZero();

  double rz = 1.0 / p[2];

  keypoint_t kp;
  kp[0] = p[0] * rz;
  kp[1] = p[1] * rz;
  _distortion.distort(kp);

  J(0, 0) = kp[0];
  J(0, 2) = 1;

  J(1, 1) = kp[1];
  J(1, 3) = 1;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void PinholeProjection<DISTORTION_T>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);

  double rz = 1.0 / p[2];
  keypoint_t kp;
  kp[0] = p[0] * rz;
  kp[1] = p[1] * rz;

  _distortion.distortParameterJacobian(kp, outJd);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
  J.derived().resize(KeypointDimension, _distortion.minimalDimensions());

  J.row(0) *= _fu;
  J.row(1) *= _fv;

}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void PinholeProjection<DISTORTION_T>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JI>, 2, 4);

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
void PinholeProjection<DISTORTION_T>::homogeneousToKeypointDistortionJacobian(
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
void PinholeProjection<DISTORTION_T>::save(Archive & ar,
                                           const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_fu);
  ar << BOOST_SERIALIZATION_NVP(_fv);
  ar << BOOST_SERIALIZATION_NVP(_cu);
  ar << BOOST_SERIALIZATION_NVP(_cv);
  ar << BOOST_SERIALIZATION_NVP(_ru);
  ar << BOOST_SERIALIZATION_NVP(_rv);
  ar << BOOST_SERIALIZATION_NVP(_distortion);

}

template<typename DISTORTION_T>
template<class Archive>
void PinholeProjection<DISTORTION_T>::load(Archive & ar,
                                           const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_fu);
  ar >> BOOST_SERIALIZATION_NVP(_fv);
  ar >> BOOST_SERIALIZATION_NVP(_cu);
  ar >> BOOST_SERIALIZATION_NVP(_cv);
  ar >> BOOST_SERIALIZATION_NVP(_ru);
  ar >> BOOST_SERIALIZATION_NVP(_rv);
  ar >> BOOST_SERIALIZATION_NVP(_distortion);

  updateTemporaries();
}

// \brief creates a random valid keypoint.
template<typename DISTORTION_T>
Eigen::VectorXd PinholeProjection<DISTORTION_T>::createRandomKeypoint() const {
  Eigen::VectorXd out(2, 1);
  out.setRandom();
  out(0) = fabs(out(0)) * _ru;
  out(1) = fabs(out(1)) * _rv;
  return out;
}

// \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
template<typename DISTORTION_T>
Eigen::Vector3d PinholeProjection<DISTORTION_T>::createRandomVisiblePoint(
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
bool PinholeProjection<DISTORTION_T>::isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  return keypoint[0] >= 0 && keypoint[1] >= 0 && keypoint[0] < (double) _ru
      && keypoint[1] < (double) _rv;

}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool PinholeProjection<DISTORTION_T>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> & p) const {
  keypoint_t k;
  return euclideanToKeypoint(p, k);

}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool PinholeProjection<DISTORTION_T>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> & ph) const {
  keypoint_t k;
  return homogeneousToKeypoint(ph, k);
}

template<typename DISTORTION_T>
void PinholeProjection<DISTORTION_T>::update(const double * v) {
  _fu += v[0];
  _fv += v[1];
  _cu += v[2];
  _cv += v[3];
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;
}

template<typename DISTORTION_T>
int PinholeProjection<DISTORTION_T>::minimalDimensions() const {
  return IntrinsicsDimension;
}

template<typename DISTORTION_T>
void PinholeProjection<DISTORTION_T>::getParameters(Eigen::MatrixXd & P) const {
  P.resize(4, 1);
  P(0, 0) = _fu;
  P(1, 0) = _fv;
  P(2, 0) = _cu;
  P(3, 0) = _cv;
}

template<typename DISTORTION_T>
void PinholeProjection<DISTORTION_T>::setParameters(const Eigen::MatrixXd & P) {
  _fu = P(0, 0);
  _fv = P(1, 0);
  _cu = P(2, 0);
  _cv = P(3, 0);
  updateTemporaries();
}

template<typename DISTORTION_T>
Eigen::Vector2i PinholeProjection<DISTORTION_T>::parameterSize() const {
  return Eigen::Vector2i(4, 1);
}

template<typename DISTORTION_T>
void PinholeProjection<DISTORTION_T>::updateTemporaries() {
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;

}

template<typename DISTORTION_T>
bool PinholeProjection<DISTORTION_T>::isBinaryEqual(
    const PinholeProjection<DISTORTION_T> & rhs) const {
  return _fu == rhs._fu && _fv == rhs._fv && _cu == rhs._cu && _cv == rhs._cv
      && _ru == rhs._ru && _rv == rhs._rv && _recip_fu == rhs._recip_fu
      && _recip_fv == rhs._recip_fv && _fu_over_fv == rhs._fu_over_fv
      && _distortion.isBinaryEqual(rhs._distortion);
}

template<typename DISTORTION_T>
PinholeProjection<DISTORTION_T> PinholeProjection<DISTORTION_T>::getTestProjection() {
  return PinholeProjection<DISTORTION_T>(400, 400, 320, 240, 640, 480,
                                         DISTORTION_T::getTestDistortion());
}

template<typename DISTORTION_T>
void PinholeProjection<DISTORTION_T>::resizeIntrinsics(double scale) {
  _fu *= scale;
  _fv *= scale;
  _cu *= scale;
  _cv *= scale;
  _ru = _ru * scale;
  _rv = _rv * scale;

  updateTemporaries();
}

/// \brief Get a set of border rays
template<typename DISTORTION_T>
void PinholeProjection<DISTORTION_T>::getBorderRays(Eigen::MatrixXd & rays) {
  rays.resize(4, 8);
  keypointToHomogeneous(Eigen::Vector2d(0.0, 0.0), rays.col(0));
  keypointToHomogeneous(Eigen::Vector2d(0.0, _rv * 0.5), rays.col(1));
  keypointToHomogeneous(Eigen::Vector2d(0.0, _rv - 1.0), rays.col(2));
  keypointToHomogeneous(Eigen::Vector2d(_ru - 1.0, 0.0), rays.col(3));
  keypointToHomogeneous(Eigen::Vector2d(_ru - 1.0, _rv * 0.5), rays.col(4));
  keypointToHomogeneous(Eigen::Vector2d(_ru - 1.0, _rv - 1.0), rays.col(5));
  keypointToHomogeneous(Eigen::Vector2d(_ru * 0.5, 0.0), rays.col(6));
  keypointToHomogeneous(Eigen::Vector2d(_ru * 0.5, _rv - 1.0), rays.col(7));

}

class PinholeHelpers {
public:

  static inline double square(double x) {
    return x*x;
  }
  static inline float square(float x) {
    return x*x;
  }
  static inline double hypot(double a, double b) {
    return sqrt(square(a) + square(b));
  }

  static std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                            double x2, double y2, double r2) {
    std::vector<cv::Point2d> ipts;

    double d = hypot(x1-x2, y1-y2);
    if (d > r1 + r2) {
      // circles are separate
      return ipts;
    }
    if (d < fabs(r1 - r2)) {
      // one circle is contained within the other
      return ipts;
    }

    double a = (square(r1) - square(r2) + square(d)) / (2.0 * d);
    double h = sqrt(square(r1) - square(a));

    double x3 = x1 + a * (x2 - x1) / d;
    double y3 = y1 + a * (y2 - y1) / d;

    if (h < 1e-10) {
      // two circles touch at one point
      ipts.push_back(cv::Point2d(x3, y3));
      return ipts;
    }

    ipts.push_back(cv::Point2d(x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d));
    ipts.push_back(cv::Point2d(x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d));
    return ipts;
  }

  static void fitCircle(const std::vector<cv::Point2d>& points, double& centerX,
                 double& centerY, double& radius) {
    // D. Umbach, and K. Jones, A Few Methods for Fitting Circles to Data,
    // IEEE Transactions on Instrumentation and Measurement, 2000
    // We use the modified least squares method.
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double sum_xxx = 0.0;
    double sum_xxy = 0.0;
    double sum_xyy = 0.0;
    double sum_yyy = 0.0;

    int n = points.size();
    for (int i = 0; i < n; ++i) {
      double x = points.at(i).x;
      double y = points.at(i).y;

      sum_x += x;
      sum_y += y;
      sum_xx += x * x;
      sum_xy += x * y;
      sum_yy += y * y;
      sum_xxx += x * x * x;
      sum_xxy += x * x * y;
      sum_xyy += x * y * y;
      sum_yyy += y * y * y;
    }

    double A = n * sum_xx - square(sum_x);
    double B = n * sum_xy - sum_x * sum_y;
    double C = n * sum_yy - square(sum_y);
    double D = 0.5
        * (n * sum_xyy - sum_x * sum_yy + n * sum_xxx - sum_x * sum_xx);
    double E = 0.5
        * (n * sum_xxy - sum_y * sum_xx + n * sum_yyy - sum_y * sum_yy);

    centerX = (D * C - B * E) / (A * C - square(B));
    centerY = (A * E - B * D) / (A * C - square(B));

    double sum_r = 0.0;
    for (int i = 0; i < n; ++i) {
      double x = points.at(i).x;
      double y = points.at(i).y;
      sum_r += hypot(x - centerX, y - centerY);
    }
    radius = sum_r / n;
  }

  static double medianOfVectorElements(std::vector<double> values)
  {
    double median;
    size_t size = values.size();
    std::sort(values.begin(), values.end());
    if (size%2 == 0)
        median = (values[size / 2 - 1] + values[size / 2]) / 2;
    else
        median = values[size / 2];
    return median;
  }

}; // class PinholeHelpers


/// \brief initialize the intrinsics based on one view of a gridded calibration target
/// These functions are based on functions from Lionel Heng and the excellent camodocal
/// https://github.com/hengli/camodocal
//this algorithm can be used with high distortion lenses
template<typename DISTORTION_T>
bool PinholeProjection<DISTORTION_T>::initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations) {

  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
  SM_ASSERT_TRUE(Exception, observations.size() != 0, "Need min. one observation");

  // First, initialize the image center at the center of the image.
  _cu = (observations[0].imCols() - 1.0) / 2.0;
  _cv = (observations[0].imRows() - 1.0) / 2.0;
  _ru = observations[0].imCols();
  _rv = observations[0].imRows();
  _distortion.clear();

  //process all images
  size_t nImages = observations.size();

  // Initialize focal length
  // C. Hughes, P. Denny, M. Glavin, and E. Jones,
  // Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
  // Extraction, PAMI 2010
  // Find circles from rows of chessboard corners, and for each pair
  // of circles, find vanishing points: v1 and v2.
  // f = ||v1 - v2|| / PI;
  std::vector<double> f_guesses;

  for (size_t i=0; i<nImages; ++i) {
    const GridCalibrationTargetObservation& obs = observations.at(i);
    SM_ASSERT_TRUE(Exception, obs.target(), "The GridCalibrationTargetObservation has no target object");
    const GridCalibrationTargetBase & target = *obs.target();

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> center(target.rows());
    double radius[target.rows()];
    bool skipImage=false;

    for (size_t r=0; r<target.rows(); ++r) {
      std::vector<cv::Point2d> circle;
      for (size_t c=0; c<target.cols(); ++c) {
        Eigen::Vector2d imagePoint;
        Eigen::Vector3d gridPoint;

        if (obs.imageGridPoint(r, c, imagePoint))
          circle.push_back(cv::Point2f(imagePoint[0], imagePoint[1]));
        else
          //skip this image if the board view is not complete
          skipImage=true;
      }
      PinholeHelpers::fitCircle(circle, center[r](0), center[r](1), radius[r]);
    }

    if(skipImage)
      continue;

    for (size_t j=0; j<target.rows(); ++j)
    {
      for (size_t k=j+1; k<target.cols(); ++k)
      {
        // find distance between pair of vanishing points which
        // correspond to intersection points of 2 circles
        std::vector < cv::Point2d > ipts;
        ipts = PinholeHelpers::intersectCircles(center[j](0), center[j](1),
                                                radius[j], center[k](0), center[k](1), radius[k]);
        if (ipts.size()<2)
          continue;

         double f_guess = cv::norm(ipts.at(0) - ipts.at(1)) / M_PI;
         f_guesses.push_back(f_guess);
      }
    }
  }

  //get the median of the guesses
  if(f_guesses.empty())
    return false;
  double f0 = PinholeHelpers::medianOfVectorElements(f_guesses);

  //set the estimate
  _fu = f0;
  _fv = f0;
  updateTemporaries();

  return true;
}

template<typename DISTORTION_T>
size_t PinholeProjection<DISTORTION_T>::computeReprojectionError(
    const GridCalibrationTargetObservation & obs,
    const sm::kinematics::Transformation & T_target_camera,
    double & outErr) const {
  outErr = 0.0;
  size_t count = 0;
  sm::kinematics::Transformation T_camera_target = T_target_camera.inverse();

  for (size_t i = 0; i < obs.target()->size(); ++i) {
    Eigen::Vector2d y, yhat;
    if (obs.imagePoint(i, y)
        && euclideanToKeypoint(T_camera_target * obs.target()->point(i),
                               yhat)) {
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
template<typename DISTORTION_T>
bool PinholeProjection<DISTORTION_T>::estimateTransformation(
    const GridCalibrationTargetObservation & obs,
    sm::kinematics::Transformation & out_T_t_c) const {

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
    }
  }

  Ps.resize(count);
  Ms.resize(count);

  std::vector<double> distCoeffs(4, 0.0);

  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);

  if (Ps.size() < 4)
    return false;

  // Call the OpenCV pnp function.
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
