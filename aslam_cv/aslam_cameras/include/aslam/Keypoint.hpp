/**
 * @file   Keypoint.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Sat Nov 24 19:47:51 2012
 * 
 * @brief  A class template for a single image point
 * 
 * 
 */

#ifndef ASLAM_KEYPOINT_HPP
#define ASLAM_KEYPOINT_HPP

#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>
#include <Eigen/Core>
#include <aslam/KeypointBase.hpp>
#include <sm/eigen/static_assert.hpp>

namespace aslam {

template<int KEYPOINT_DIMENSION>
class Keypoint : public KeypointBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    KeypointDimension = KEYPOINT_DIMENSION /*!< The dimension of the keypoint associated with this geometry policy */
  };

  typedef Eigen::Matrix<double, KeypointDimension, 1> measurement_t;
  typedef Eigen::Matrix<double, KeypointDimension, KeypointDimension> inverse_covariance_t;

  /// \brief default constructor
  Keypoint();

  /// \brief the copy constructor makes a deep copy
  Keypoint(const Keypoint & rhs);

  /// \brief the assignment operator makes a deep copy
  Keypoint & operator=(const Keypoint & rhs);

  virtual ~Keypoint();

  /// \brief swap this keypoint with the rhs
  void swap(Keypoint & rhs);

  virtual size_t dimension() const {
    return KeypointDimension;
  }

  ///////////////////////////////////////////////////
  // The measurement and uncertainty
  ///////////////////////////////////////////////////

  /// \brief the image measurement
  const measurement_t & measurement() const;

  /// \brief the image measurement
  const measurement_t & y() const;

  /// \brief set the image measurement
  void setMeasurement(const measurement_t & m);

  /// \brief the octave this keypoint was detected in
  int octave() const;

  /// \brief set the octave this keypoint was detected in
  void setOctave(const int octave);

  /// \breif get/set the trace id
  void setTraceId(const size_t traceId);
  size_t getTraceId();

  /// \brief compute the mahalonibis distance between y and this keypoint
  template<typename DERIVED>
  double mahalonibisDistance(const Eigen::MatrixBase<DERIVED> & y) const;

  /// \brief get the image measurement inverse covariance
  const inverse_covariance_t & inverseMeasurementCovariance() const;

  /// \brief get the image measurement inverse covariance
  const inverse_covariance_t & invR() const;

  /// \brief set the image measurement inverse covariance
  void setInverseMeasurementCovariance(const inverse_covariance_t & invR);
  void setInverseCovariance(const inverse_covariance_t & invR) {
    setInverseMeasurementCovariance(invR);
  }

  /// \brief get the measurement
  virtual Eigen::VectorXd vsMeasurement() const;
  /// \brief set the measurement
  virtual void vsSetMeasurement(const Eigen::VectorXd & m);

  /// \brief get the inverse measurement covariance
  virtual Eigen::MatrixXd vsInverseMeasurementCovariance() const;
  /// \brief set the inverse measurement covariance.
  virtual void vsSetInverseMeasurementCovariance(const Eigen::MatrixXd & invR);

  virtual double vsMahalonibisDistance(const Eigen::VectorXd & y) const;

  ///////////////////////////////////////////////////
  // The descriptor
  ///////////////////////////////////////////////////        

  /// \brief get the keypoint descriptor
  virtual const DescriptorBase * descriptor() const;

  /// \brief get the keypoint descriptor
  virtual boost::shared_ptr<DescriptorBase> descriptorPtr() const;

  /// \brief set the keypoint descriptor
  virtual void setDescriptor(const DescriptorBase & descriptor);

  /// \brief set the keypoint descriptor
  ///        the keypoint takes ownership of the pointer
  virtual void setDescriptorRawPtr(DescriptorBase * descriptor);

  /// \brief set the keypoint descriptor
  ///        the keypoint takes ownership of the pointer
  virtual void setDescriptorPtr(boost::shared_ptr<DescriptorBase> descriptor);

  ///////////////////////////////////////////////////
  // The landmark
  ///////////////////////////////////////////////////

  /// \brief get the landmark
  virtual const sm::kinematics::UncertainHomogeneousPoint & landmark() const;
  virtual const sm::kinematics::UncertainHomogeneousPoint * landmarkPtr() const;

  /// \brief set the landmark
  virtual void setLandmark(
      const sm::kinematics::UncertainHomogeneousPoint & landmark);

  /// \brief get the landmark id
  virtual const LandmarkId & landmarkId() const;

  /// \brief set the landmark id
  virtual void setLandmarkId(const LandmarkId & landmarkId);

  /// \brief is the landmark initialized
  virtual bool isLandmarkInitialized() const;

  /// \brief reset and clear the landmark.
  virtual void clearLandmark();

  ///////////////////////////////////////////////////
  // The back projection
  ///////////////////////////////////////////////////

  /// \brief set the back projection
  virtual void setBackProjection(const sm::kinematics::UncertainVector3 & v);

  /// \brief set the back projection
  virtual void setBackProjectionVector(const Eigen::Vector3d & v);

  /// \brief get the back projection pointer
  virtual const sm::kinematics::UncertainVector3 * backProjection() const;

  /// \brief is the back projection initialized?
  virtual bool isBackProjectionSet() const;

  /// \brief clear the back projection
  virtual void clearBackProjection();

  ///////////////////////////////////////////////////
  // Serialization
  ///////////////////////////////////////////////////

  enum {
    CLASS_SERIALIZATION_VERSION = 1
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int) CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");

    ar >> BOOST_SERIALIZATION_NVP(_measurement);
    ar >> BOOST_SERIALIZATION_NVP(_inverseMeasurementCovariance);
    ar >> BOOST_SERIALIZATION_NVP(_landmarkId);
    ar >> BOOST_SERIALIZATION_NVP(_landmark);
    ar >> BOOST_SERIALIZATION_NVP(_descriptor);
    ar >> BOOST_SERIALIZATION_NVP(_backProjection);
    if (version > 0) {
      ar >> BOOST_SERIALIZATION_NVP(_octave);
    }
  }

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(_measurement);
    ar << BOOST_SERIALIZATION_NVP(_inverseMeasurementCovariance);
    ar << BOOST_SERIALIZATION_NVP(_landmarkId);
    ar << BOOST_SERIALIZATION_NVP(_landmark);
    ar << BOOST_SERIALIZATION_NVP(_descriptor);
    ar << BOOST_SERIALIZATION_NVP(_backProjection);
    ar << BOOST_SERIALIZATION_NVP(_octave);
  }

  /// \brief Support for unit testing. Is this keypoint binary equal to the other?
  bool isBinaryEqual(const Keypoint<KEYPOINT_DIMENSION> & rhs) const;

  /// \brief Support for unit testing. Set this to a random keypoint.
  void setRandom();
 private:

  /// \brief Frees all memory owned by the keypoint.
  void freeAll();

  /// \brief The keypoint measurement
  measurement_t _measurement;

  /// \brief The inverse covariance measurement
  inverse_covariance_t _inverseMeasurementCovariance;

  /// \brief the landmark id.
  LandmarkId _landmarkId;

  /// \brief the landmark.
  sm::kinematics::UncertainHomogeneousPoint * _landmark;

  /// \brief the keypoint descriptor
  boost::shared_ptr<DescriptorBase> _descriptor;

  /// \brief the back projection, a ray in space originating from the view origin.
  sm::kinematics::UncertainVector3 * _backProjection;

  /// \brief the image octave this keypoint was observed in
  unsigned int _octave;

  /// \brief unique ground truth ID for a keypoint trace (i.e. to easily verifiy 2D2D matching with ground truth simulation)
  size_t _traceId;
};

}  // namespace aslam

#include <aslam/implementation/Keypoint.hpp>

// this is a fancy templated version of 
// BOOST_CLASS_VERSION() for serialization versioning
SM_BOOST_CLASS_VERSION_I1 (aslam::Keypoint);

#endif /* ASLAM_KEYPOINT_HPP */
