#ifndef ASLAM_CAMERA_GEOMETRY_HPP
#define ASLAM_CAMERA_GEOMETRY_HPP

#include <aslam/cameras/CameraGeometryBase.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <boost/serialization/nvp.hpp>
#include "FiniteDifferences.hpp"
#include <sm/PropertyTree.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>
#include <aslam/Frame.hpp>

namespace aslam {
// forward declaration
template<typename C>
class Frame;

namespace cameras {

/**
 * \class CameraGeometry
 * 
 * A camera geometry class based on three configurable policies:
 * 1. a lens type that determines the geometry of the camera
 * 2. a shutter type that determines the timing of each pixel.
 * 3. a mask type.
 *
 */
template<typename PROJECTION_T, typename SHUTTER_T, typename MASK_T>
class CameraGeometry : public CameraGeometryBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef PROJECTION_T projection_t;
  // Not all projections will have a distortion...is this necessary? Maybe.
  //typedef typename projection_t::distortion_t distortion_t;
  typedef typename projection_t::keypoint_t keypoint_t;
  typedef SHUTTER_T shutter_t;
  typedef MASK_T mask_t;
  typedef aslam::Frame<CameraGeometry<projection_t, shutter_t, mask_t> > frame_t;

  enum {
    KeypointDimension = projection_t::KeypointDimension
  };

  typedef Eigen::Matrix<double, KeypointDimension, 4> jacobian_homogeneous_t;
  typedef Eigen::Matrix<double, KeypointDimension, 3> jacobian_t;
  typedef Eigen::Matrix<double, 3, KeypointDimension> inverse_jacobian_t;
  typedef Eigen::Matrix<double, 4, KeypointDimension> inverse_jacobian_homogeneous_t;
  typedef Eigen::Matrix<double, KeypointDimension, KeypointDimension> covariance_t;

  /// \brief default constructor
  CameraGeometry();

  /// \brief Construct from projection
  CameraGeometry(const projection_t & projection);

  /// \brief Construct from projection and shutter
  CameraGeometry(const projection_t & projection, const shutter_t & shutter);

  /// \brief Construct from projection, shutter, and mask
  CameraGeometry(const projection_t & projection, const shutter_t & shutter,
                 const mask_t & mask);

  /// \brief Construct from a property tree
  CameraGeometry(const sm::PropertyTree & config);

  /// \brief simple destructor
  virtual ~CameraGeometry();

  //////////////////////////////////////////////////////////////
  // PROJECTION FUNCTIONS
  //////////////////////////////////////////////////////////////
  template<typename DERIVED_P, typename DERIVED_K>
  bool euclideanToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                           const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                           const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypointFiniteDifference(
      const Eigen::MatrixBase<DERIVED_P> & p,
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

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool homogeneousToKeypointFiniteDifference(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
      const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  // Project the point and get the associated uncertainty of the projection.
  template<typename DERIVED_K>
  bool homogeneousToKeypoint(
      const sm::kinematics::UncertainHomogeneousPoint & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
      covariance_t & outProjectionUncertainty) const;

  // Project the point
  template<typename DERIVED_K>
  bool homogeneousToKeypoint(
      const sm::kinematics::HomogeneousPoint & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint,
                           const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToEuclideanFiniteDifference(
      const Eigen::MatrixBase<DERIVED_K> & keypoint,
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

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToHomogeneousFiniteDifference(
      const Eigen::MatrixBase<DERIVED_K> & keypoint,
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
  void euclideanToKeypointIntrinsicsJacobianFiniteDifference(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void euclideanToKeypointDistortionJacobianFiniteDifference(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void homogeneousToKeypointIntrinsicsJacobianFiniteDifference(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JS>
  void homogeneousToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JS> & outJs) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void homogeneousToKeypointDistortionJacobianFiniteDifference(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  //////////////////////////////////////////////////////////////
  // SHUTTER SUPPORT
  //////////////////////////////////////////////////////////////

  // The amount of time elapsed between the start of the image and the
  // keypoint. For a global shutter camera, this can return Duration(0).
  template<typename DERIVED_K>
  Duration temporalOffset(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  virtual Duration temporalOffset(const Eigen::VectorXd & keypoint) const;

  //////////////////////////////////////////////////////////////
  // VALIDITY TESTING
  //////////////////////////////////////////////////////////////

  template<typename DERIVED_K>
  bool isValid(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  template<typename DERIVED_P>
  bool isEuclideanVisible(const Eigen::MatrixBase<DERIVED_P> & p) const;

  template<typename DERIVED_P>
  bool isHomogeneousVisible(const Eigen::MatrixBase<DERIVED_P> & ph) const;

  //////////////////////////////////////////////////////////////
  // SUPERCLASS SUPPORT
  //////////////////////////////////////////////////////////////
  virtual bool vsEuclideanToKeypoint(const Eigen::Vector3d & p,
                                     Eigen::VectorXd & outKeypoint) const;
  virtual bool vsEuclideanToKeypoint(const Eigen::Vector3d & p,
                                     Eigen::VectorXd & outKeypoint,
                                     Eigen::MatrixXd & outJacobian) const;

  virtual bool vsHomogeneousToKeypoint(const Eigen::Vector4d & ph,
                                       Eigen::VectorXd & outKeypoint) const;

  virtual bool vsHomogeneousToKeypoint(const Eigen::Vector4d & ph,
                                       Eigen::VectorXd & outKeypoint,
                                       Eigen::MatrixXd & outJacobian) const;

  virtual bool vsKeypointToEuclidean(const Eigen::VectorXd & keypoint,
                                     Eigen::Vector3d & outPoint) const;

  virtual bool vsKeypointToEuclidean(const Eigen::VectorXd & keypoint,
                                     Eigen::VectorXd & outPoint,
                                     Eigen::MatrixXd & outJacobian) const;

  virtual bool vsKeypointToHomogeneous(Eigen::VectorXd const & keypoint,
                                       Eigen::VectorXd & outPoint) const;

  virtual bool vsKeypointToHomogeneous(Eigen::VectorXd const & keypoint,
                                       Eigen::VectorXd & outPoint,
                                       Eigen::MatrixXd & outJacobian) const;

  virtual bool vsIsValid(const Eigen::VectorXd & keypoint) const;

  virtual bool vsIsEuclideanVisible(const Eigen::Vector3d & p) const;

  virtual bool vsIsHomogeneousVisible(const Eigen::Vector4d & ph) const;

  // The amount of time elapsed between the start of the image and the
  // keypoint. For a global shutter camera, this can return Duration(0).
  virtual Duration vsTemporalOffset(const Eigen::VectorXd & keypoint) const;

  /// \brief get the camera id.
  virtual CameraId id() const;

  /// \brief set the camera id.
  virtual void setId(CameraId id);

  /// \brief the length of a keypoint
  virtual size_t keypointDimension() const;

  /// \brief is this an invertible camera model
  virtual bool isProjectionInvertible() const;

  //////////////////////////////////////////////////////////////
  // CAMERA CALIBRATION
  //////////////////////////////////////////////////////////////
  /// \brief initialize the intrinsics based on a list of views of a gridded calibration target
  /// \return true on success
  bool initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations);

  /// \brief estimate the transformation of the camera with respect to the calibration target
  ///        On success out_T_t_c is filled in with the transformation that takes points from
  ///        the camera frame to the target frame
  /// \return true on success
  virtual bool estimateTransformation(
      const GridCalibrationTargetObservation & obs,
      sm::kinematics::Transformation & out_T_t_c) const;

  ////////////////////////////////////////////////////////////////
  // OPTIMIZATION SUPPORT
  ////////////////////////////////////////////////////////////////
  /// \brief the minimal dimensions of the projection parameters
  virtual int minimalDimensionsProjection() const;

  /// \brief the minimal dimensions of the distortion parameters
  virtual int minimalDimensionsDistortion() const;

  /// \brief the minimal dimensions of the shutter parameters
  virtual int minimalDimensionsShutter() const;

  // aslam::backend compatibility

  /// \brief update the intrinsics
  virtual void update(const double * v, bool projection, bool distortion,
                      bool shutter);

  /// \brief Get the total number of dimensions of the intrinsic parameters
  virtual int minimalDimensions(bool projection, bool distortion,
                                bool shutter) const;

  /// \brief get the intrinsic parameters.
  virtual void getParameters(Eigen::MatrixXd & P, bool projection,
                             bool distortion, bool shutter) const;

  /// \brief set the intrinsic parameters.
  virtual void setParameters(const Eigen::MatrixXd & P, bool projection,
                             bool distortion, bool shutter);

  /// \brief return the Jacobian of the projection with respect to the intrinsics.
  virtual void euclideanToKeypointIntrinsicsJacobian(
      const Eigen::Vector3d & p, Eigen::MatrixXd & outJi,
      bool estimateProjection, bool estimateDistortion,
      bool estimateShutter) const;

  /// \brief return the Jacobian of the projection with respect to the intrinsics.
  virtual void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::Vector4d & p, Eigen::MatrixXd & outJi,
      bool estimateProjection, bool estimateDistortion,
      bool estimateShutter) const;

  /// \brief return the temporal offset with respect to the intrinsics.
  virtual void temporalOffsetIntrinsicsJacobian(
      const Eigen::VectorXd & keypoint, Eigen::MatrixXd & outJi,
      bool estimateProjection, bool estimateDistortion,
      bool estimateShutter) const;

  //////////////////////////////////////////////////////////////
  // UNIT TEST SUPPORT
  //////////////////////////////////////////////////////////////
  // \brief creates a random valid keypoint.
  virtual Eigen::VectorXd createRandomKeypoint() const;

  // \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const;

  projection_t & projection() {
    return _projection;
  }
  const projection_t & projection() const {
    return _projection;
  }

  shutter_t & shutter() {
    return _shutter;
  }
  const shutter_t & shutter() const {
    return _shutter;
  }

  void setMask(const mask_t & mask) {
    _mask = mask;
  }
  mask_t & mask() {
    return _mask;
  }
  const mask_t & mask() const {
    return _mask;
  }

  virtual bool hasMask() const
  {
	  return _mask.isSet();
  }

  virtual void print(std::ostream & out);

  bool isBinaryEqual(
      const CameraGeometry<PROJECTION_T, SHUTTER_T, MASK_T> & rhs) const;

  /// \brief This will create a frame with the right type. WARNING: it will not
  ///        fill in the camera geometry in the frame. This must be done manually
  ///        outside the call.            
  virtual boost::shared_ptr<FrameBase> createUninitializedFrameBase() const;

  /// \brief This will create a frame with the right type. WARNING: it will not
  ///        fill in the camera geometry in the frame. This must be done manually
  ///        outside the call.
  boost::shared_ptr<frame_t> createUninitializedFrame() const;

  enum {
    CLASS_SERIALIZATION_VERSION = 3
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int) CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");
    if (version >= 2) {
      ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(CameraGeometryBase);
    }

    ar >> BOOST_SERIALIZATION_NVP(_id);
    ar >> BOOST_SERIALIZATION_NVP(_projection);
    ar >> BOOST_SERIALIZATION_NVP(_shutter);
    ar >> BOOST_SERIALIZATION_NVP(_mask);

  }

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(CameraGeometryBase);
    ar << BOOST_SERIALIZATION_NVP(_id);
    ar << BOOST_SERIALIZATION_NVP(_projection);
    ar << BOOST_SERIALIZATION_NVP(_shutter);
    ar << BOOST_SERIALIZATION_NVP(_mask);
  }

  static CameraGeometry<PROJECTION_T, SHUTTER_T, MASK_T> getTestGeometry();

  /// \todo redo this. Somehow Stefan's stuff needs it.
  virtual int width() const {
    return _projection.ru();
  }
  virtual int height() const {
    return _projection.rv();
  }
 private:
  CameraId _id;
  projection_t _projection;
  shutter_t _shutter;
  mask_t _mask;
};

}  // namespace cameras
}  // namespace aslam

#include "implementation/CameraGeometry.hpp"

SM_BOOST_CLASS_VERSION_T1T2T3 (aslam::cameras::CameraGeometry);

#endif /* ASLAM_CAMERA_GEOMETRY_BASE_HPP */
