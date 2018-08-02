#ifndef ASLAM_CAMERAS_DOUBLE_SPHERE_PROJECTION_HPP
#define ASLAM_CAMERAS_DOUBLE_SPHERE_PROJECTION_HPP

#include "StaticAssert.hpp"
#include "FiniteDifferences.hpp"
#include <sm/PropertyTree.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sm/logging.hpp>

namespace aslam {
namespace cameras {

template<typename DISTORTION_T>
class DoubleSphereProjection {
 public:

  enum {
    KeypointDimension = 2
  };
  enum {
    IntrinsicsDimension = 6
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  typedef DISTORTION_T distortion_t;
  typedef Eigen::Matrix<double, KeypointDimension, 1> keypoint_t;
  typedef Eigen::Matrix<double, KeypointDimension, IntrinsicsDimension> jacobian_intrinsics_t;

  /// \brief Default constructor
  DoubleSphereProjection();

  DoubleSphereProjection(double xi, double alpha, double focalLengthU, double focalLengthV,
                 double imageCenterU, double imageCenterV, int resolutionU,
                 int resolutionV, distortion_t distortion);

  DoubleSphereProjection(double xi, double alpha, double focalLengthU, double focalLengthV,
                 double imageCenterU, double imageCenterV, int resolutionU,
                 int resolutionV);

  DoubleSphereProjection(const sm::PropertyTree & config);

  /// \brief destructor.
  virtual ~DoubleSphereProjection();

  /// \brief resize the intrinsics based on a scaling of the image.
  void resizeIntrinsics(double scale);

  template<typename DERIVED_P, typename DERIVED_K>
  bool euclideanToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                           const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                           const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_P, typename DERIVED_K>
  bool homogeneousToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool homogeneousToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                             const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                             const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint,
                           const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToHomogeneous(
      const Eigen::MatrixBase<DERIVED_K> & keypoint,
      const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToHomogeneous(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                             const Eigen::MatrixBase<DERIVED_P> & outPoint,
                             const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void euclideanToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void euclideanToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void homogeneousToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_K>
  bool isValid(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  template<typename DERIVED_K>
  bool isLiftable(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  bool isUndistortedKeypointValid(const double rho2_d) const;

  template<typename DERIVED_P>
  bool isEuclideanVisible(const Eigen::MatrixBase<DERIVED_P> & p) const;

  template<typename DERIVED_P>
  bool isHomogeneousVisible(const Eigen::MatrixBase<DERIVED_P> & ph) const;

  /// \brief initialize the intrinsics based on one view of a gridded calibration target
  /// \return true on success
  bool initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations);

  /// \brief estimate the transformation of the camera with respect to the calibration target
  ///        On success out_T_t_c is filled in with the transformation that takes points from
  ///        the camera frame to the target frame
  /// \return true on success
  bool estimateTransformation(const GridCalibrationTargetObservation & obs,
                              sm::kinematics::Transformation & out_T_t_c) const;

  /// \brief compute the reprojection error based on a checkerboard observation.
  /// \return the number of corners successfully observed and projected
  size_t computeReprojectionError(
      const GridCalibrationTargetObservation & obs,
      const sm::kinematics::Transformation & T_target_camera,
      double & outErr) const;

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();
  template<class Archive>
  void load(Archive & ar, const unsigned int version);
  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  /// \brief creates a random valid keypoint.
  virtual Eigen::VectorXd createRandomKeypoint() const;

  /// \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const;

  /// \brief is the projection invertible?
  bool isProjectionInvertible() const {
    return true;
  }

  /// \brief set the distortion model.
  void setDistortion(const distortion_t & distortion) {
    _distortion = distortion;
  }
  /// \brief returns a reference to the distortion model
  distortion_t & distortion() {
    return _distortion;
  }
  ;
  const distortion_t & distortion() const {
    return _distortion;
  }
  ;

  Eigen::Matrix3d getCameraMatrix() {
    Eigen::Matrix3d K;
    K << _fu, 0.0, _cu, 0.0, _fv, _cv, 0.0, 0.0, 1.0;
    return K;
  }
  ;

  /// \brief the xi parameter corresponding to distance between spheres.
  double xi() const {
    return _xi;
  }
  /// \brief the alpha parameter that relates second sphere and pinhole projection.
  double alpha() const {
    return _alpha;
  }
  /// \brief The horizontal focal length in pixels.
  double fu() const {
    return _fu;
  }
  /// \brief The vertical focal length in pixels.
  double fv() const {
    return _fv;
  }
  /// \brief The horizontal image center in pixels.
  double cu() const {
    return _cu;
  }
  /// \brief The vertical image center in pixels.
  double cv() const {
    return _cv;
  }
  /// \brief The horizontal resolution in pixels.
  int ru() const {
    return _ru;
  }
  /// \brief The vertical resolution in pixels.
  int rv() const {
    return _rv;
  }
  /// \brief The horizontal resolution in pixels.
  int width() const {
    return _ru;
  }
  /// \brief The vertical resolution in pixels.
  int height() const {
    return _rv;
  }

  double focalLengthCol() const {
    return _fu;
  }
  double focalLengthRow() const {
    return _fv;
  }
  double opticalCenterCol() const {
    return _cu;
  }
  double opticalCenterRow() const {
    return _cv;
  }

  int keypointDimension() const {
    return KeypointDimension;
  }

  bool isBinaryEqual(const DoubleSphereProjection<distortion_t> & rhs) const;

  static DoubleSphereProjection<distortion_t> getTestProjection();
 private:

  /// \brief the xi parameter corresponding to distance between spheres.
  double _xi;
  /// \brief the alpha parameter that relates second sphere and pinhole projection.
  double _alpha;
  /// \brief The horizontal focal length in pixels.
  double _fu;
  /// \brief The vertical focal length in pixels.
  double _fv;
  /// \brief The horizontal image center in pixels.
  double _cu;
  /// \brief The vertical image center in pixels.
  double _cv;
  /// \brief The horizontal resolution in pixels.
  int _ru;
  /// \brief The vertical resolution in pixels.
  int _rv;

  /// \brief A computed value for speeding up computation.
  void updateTemporaries();

  double _recip_fu;
  double _recip_fv;
  double _fu_over_fv;
  double _one_over_2alpha_m_1;
  double _fov_parameter;  //!< parameter related to the valid region of the camera model

  distortion_t _distortion;

};

}  // namespace cameras
}  // namespace aslam

#include "implementation/DoubleSphereProjection.hpp"

SM_BOOST_CLASS_VERSION_T1 (aslam::cameras::DoubleSphereProjection);

#endif /* ASLAM_CAMERAS_DOUBLE_SPHERE_PROJECTION_HPP */
