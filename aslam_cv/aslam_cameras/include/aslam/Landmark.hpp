#ifndef _ASLAM_LANDMARK_H_
#define _ASLAM_LANDMARK_H_

#include <boost/serialization/shared_ptr.hpp>

#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <sm/boost/serialization.hpp>

#include <aslam/frontend_ids.hpp>
#include <aslam/KeypointIdentifier.hpp>
#include <aslam/DescriptorBase.hpp>

#include <boost/shared_ptr.hpp>

namespace sm {
namespace kinematics {
class Transformation;
}  // namespace kinematics
}  // namespace sm

namespace aslam {
class KeypointBase;

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<Landmark> Ptr;
  // Basics

  /// \brief default constructor
  Landmark();

  /// \brief the copy constructor makes a deep copy
  Landmark(const Landmark& rhs);

  /// \brief the assignment operator makes a deep copy
  Landmark& operator=(const Landmark& rhs);

  virtual ~Landmark();

  /// \brief swap this keypoint with the rhs
  void swap(Landmark& rhs);

  void fromKeypoint(boost::uint64_t frameId, const aslam::KeypointBase & kp);
  void fromTransformedKeypoint(boost::uint64_t frameId, const KeypointBase & kp,
                               const sm::kinematics::Transformation & T_v_c);

  // Descriptor
  /// get the Descriptor for the Landmark.
  const DescriptorBase* descriptor() const;

  /// get the Descriptor for the Landmark.
  boost::shared_ptr<DescriptorBase> descriptorPtr();

  /// \brief set the Descriptor for the Landmark.
  void setDescriptor(const DescriptorBase& descriptor);

  /// \brief set the Descriptor for the Landmark,
  ///        the Landmark takes ownership.
  void setDescriptorPtr(boost::shared_ptr<DescriptorBase> descriptor);

  /// \brief set the Descriptor for the Landmark,
  ///        the Landmark takes ownership.
  void setDescriptorRawPtr(DescriptorBase* descriptor);

  // UncertainHomogeneousPoint
  /// \brief get the UncertainHomogeneousPoint, relative to the associated frame (see frameId())
  const sm::kinematics::UncertainHomogeneousPoint& point() const;

  /// \brief set the UncertainHomogeneousPoint, relative to the associated frame (see frameId())
  void setPoint(const sm::kinematics::UncertainHomogeneousPoint& ph);

  double * pptr() {
    return _ph.pptr();
  }

  Eigen::Vector3d toEuclidean() const;
  Eigen::Vector4d toHomogeneous() const;

  // LandmarkId
  /// \brief get the Landmark id.
  const LandmarkId& landmarkId() const;

  /// \brief set the Landmark id.
  void setLandmarkId(const LandmarkId& landmarkId);

  // FrameId

  /// \brief get the frame ID that the landmark is stored wrt.
  const boost::uint64_t & frameId() const;

  /// \brief set the frame id that the landmark is stored wrt.
  void setFrameId(const boost::uint64_t & frameId);

  // Serialization

  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int) CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");

    ar >> BOOST_SERIALIZATION_NVP(_descriptor);
    ar >> BOOST_SERIALIZATION_NVP(_ph);
    ar >> BOOST_SERIALIZATION_NVP(_landmarkId);
    ar >> BOOST_SERIALIZATION_NVP(_frameId);
  }

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(_descriptor);
    ar << BOOST_SERIALIZATION_NVP(_ph);
    ar << BOOST_SERIALIZATION_NVP(_landmarkId);
    ar << BOOST_SERIALIZATION_NVP(_frameId);
  }

  /// \brief Support for unit testing. Is this landmark binary equal to the other?
  bool isBinaryEqual(const Landmark& rhs) const;

  /// \brief Support for unit testing. Set this to a random keypoint.
  void setRandom();

 private:

  /// \brief the descriptor.
  boost::shared_ptr<DescriptorBase> _descriptor;

  /// \brief the UncertainHomogeneousPoint, expressed relative to the frame (frameId())
  sm::kinematics::UncertainHomogeneousPoint _ph;

  /// \brief the Landmark id.
  LandmarkId _landmarkId;

  /// \brief the id of the frame that the landmark is stored wrt.
  boost::uint64_t _frameId;

};

}  //namespace aslam

#endif //_VCHARGE_LANDMARK_H_
