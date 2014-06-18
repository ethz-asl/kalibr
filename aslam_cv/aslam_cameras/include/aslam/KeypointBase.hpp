#ifndef ASLAM_KEYPOINT_BASE_HPP
#define ASLAM_KEYPOINT_BASE_HPP

#include <Eigen/Core>
#include <aslam/Time.hpp>
#include <aslam/frontend_ids.hpp>
#include <aslam/KeypointBase.hpp>
#include <aslam/DescriptorBase.hpp>
#include <aslam/KeypointIdentifier.hpp>
#include <sm/kinematics/UncertainVector.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <sm/serialization_macros.hpp>

namespace aslam {

// Forward declaration.
class Descriptor;

class KeypointBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeypointBase();
  virtual ~KeypointBase();

  virtual Eigen::VectorXd vsMeasurement() const = 0;
  virtual Eigen::VectorXd vsY() const {
    return vsMeasurement();
  }
  virtual void vsSetMeasurement(const Eigen::VectorXd & m) = 0;
  virtual double vsMahalonibisDistance(const Eigen::VectorXd & y) const = 0;

  virtual Eigen::MatrixXd vsInverseMeasurementCovariance() const = 0;
  virtual Eigen::MatrixXd vsInvR() const {
    return vsInverseMeasurementCovariance();
  }
  virtual void vsSetInverseMeasurementCovariance(
      const Eigen::MatrixXd & invR) = 0;

  virtual size_t dimension() const = 0;

  /// \brief get the keypoint descriptor
  virtual const DescriptorBase * descriptor() const = 0;

  /// \brief get the keypoint descriptor
  virtual boost::shared_ptr<DescriptorBase> descriptorPtr() const = 0;

  /// \brief set the keypoint descriptor
  virtual void setDescriptor(const DescriptorBase & descriptor) = 0;

  /// \brief set the keypoint descriptor
  ///        the keypoint takes ownership of the pointer
  virtual void setDescriptorRawPtr(DescriptorBase * descriptor) = 0;

  /// \brief set the keypoint descriptor
  ///        the keypoint takes ownership of the pointer
  virtual void setDescriptorPtr(
      boost::shared_ptr<DescriptorBase> descriptor) = 0;

  /// \breif get/set the trace id
  virtual void setTraceId(const size_t traceId) = 0;
  virtual size_t getTraceId() = 0;

  ///////////////////////////////////////////////////
  // The landmark
  ///////////////////////////////////////////////////

  /// \brief get the landmark
  virtual const sm::kinematics::UncertainHomogeneousPoint & landmark() const = 0;
  virtual const sm::kinematics::UncertainHomogeneousPoint * landmarkPtr() const = 0;

  /// \brief set the landmark
  virtual void setLandmark(
      const sm::kinematics::UncertainHomogeneousPoint & landmark) = 0;

  /// \brief get the landmark id
  virtual const LandmarkId & landmarkId() const = 0;

  /// \brief set the landmark id
  virtual void setLandmarkId(const LandmarkId & landmarkId) = 0;

  /// \brief is the landmark initialized
  virtual bool isLandmarkInitialized() const = 0;

  /// \brief is the landmark enabled
  bool isLandmarkEnabled() const;

  /// \brief disable the landmark
  void disableLandmark();
  /// \brief enable the landmark
  void enableLandmark();
  /// \brief reset and clear the landmark.
  virtual void clearLandmark() = 0;

  ///////////////////////////////////////////////////
  // The back projection
  ///////////////////////////////////////////////////

  /// \brief set the back projection
  virtual void setBackProjection(
      const sm::kinematics::UncertainVector3 & v) = 0;

  /// \brief set the back projection
  virtual void setBackProjectionVector(const Eigen::Vector3d & v) = 0;

  /// \brief get the back projection pointer
  virtual const sm::kinematics::UncertainVector3 * backProjection() const = 0;

  /// \brief is the back projection initialized?
  virtual bool isBackProjectionSet() const = 0;

  /// \brief clear the back projeciton
  virtual void clearBackProjection() = 0;

 private:
  bool _landmarkEnabled;
};

}  // namespace aslam

#endif /* ASLAM_KEYPOINT_BASE_HPP */
