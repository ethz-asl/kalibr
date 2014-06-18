#include <aslam/Landmark.hpp>
#include <sm/serialization_macros.hpp>
#include <aslam/KeypointBase.hpp>
#include <sm/kinematics/Transformation.hpp>

namespace aslam {

// Basics

Landmark::Landmark()
    : _frameId(-1) {
}

Landmark::Landmark(const Landmark& rhs)
    : _descriptor(rhs._descriptor),
      _ph(rhs._ph),
      _landmarkId(rhs._landmarkId),
      _frameId(rhs._frameId) {
}

Landmark& Landmark::operator=(const Landmark& rhs) {
  Landmark copy(rhs);
  this->swap(copy);
  return *this;
}

Landmark::~Landmark() {
}

void Landmark::swap(Landmark& rhs) {
  if (this != &rhs) {
    _descriptor.swap(rhs._descriptor);
    std::swap(_ph, rhs._ph);
    _landmarkId.swap(rhs._landmarkId);
    std::swap(_frameId, rhs._frameId);
  }
}

// Descriptor

const DescriptorBase* Landmark::descriptor() const {
  return _descriptor.get();
}

boost::shared_ptr<DescriptorBase> Landmark::descriptorPtr() {
  return _descriptor;
}

void Landmark::setDescriptor(const DescriptorBase& descriptor) {
  _descriptor.reset(descriptor.clone());
}

void Landmark::setDescriptorPtr(boost::shared_ptr<DescriptorBase> descriptor) {
  _descriptor = descriptor;
}

void Landmark::setDescriptorRawPtr(DescriptorBase* descriptor) {
  _descriptor.reset(descriptor);
}

// UncertainHomogeneousPoint

const sm::kinematics::UncertainHomogeneousPoint& Landmark::point() const {
  return _ph;
}

//const sm::kinematics::UncertainHomogeneousPoint* Landmark::phPtr() const
//{
//    return _ph;
//}

void Landmark::setPoint(const sm::kinematics::UncertainHomogeneousPoint& ph) {
  _ph = ph;
}

// LandmarkId

const LandmarkId& Landmark::landmarkId() const {
  return _landmarkId;
}

void Landmark::setLandmarkId(const LandmarkId& landmarkId) {
  _landmarkId = landmarkId;
}

// FrameId

const boost::uint64_t& Landmark::frameId() const {
  return _frameId;
}

void Landmark::setFrameId(const boost::uint64_t & frameId) {
  _frameId = frameId;
}

Eigen::Vector3d Landmark::toEuclidean() const {
  return _ph.toEuclidean();
}
Eigen::Vector4d Landmark::toHomogeneous() const {
  return _ph.toHomogeneous();
}

// Unit tests

bool Landmark::isBinaryEqual(const Landmark& rhs) const {
  bool isEqual = true;
  //ar >> BOOST_SERIALIZATION_NVP(_descriptor);

  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _descriptor);

  //ar >> BOOST_SERIALIZATION_NVP(_ph);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _ph);
  //ar >> BOOST_SERIALIZATION_NVP(_landmarkId);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _landmarkId);
  //ar >> BOOST_SERIALIZATION_NVP(_frameId);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _frameId);
  return isEqual;

}

void Landmark::setRandom() {
  /// TODO: Add Descriptor
  // boost::shared_ptr<DescriptorBase> _descriptor;
  //PM: Is this correct?
  _descriptor.reset();
  //_descriptor->setRandom();

  _ph.setRandom();
  _landmarkId.setRandom();
  _frameId = rand();
}

void Landmark::fromKeypoint(boost::uint64_t frameId, const KeypointBase & kp) {
  setFrameId(frameId);
  setDescriptorPtr(kp.descriptorPtr());
  setPoint(kp.landmark());
  setLandmarkId(kp.landmarkId());
}

void Landmark::fromTransformedKeypoint(
    boost::uint64_t frameId, const KeypointBase & kp,
    const sm::kinematics::Transformation & T_v_c) {
  setFrameId(frameId);
  setDescriptorPtr(kp.descriptorPtr());
  const sm::kinematics::UncertainHomogeneousPoint * pp_c = kp.landmarkPtr();
  SM_ASSERT_TRUE(std::runtime_error, pp_c != NULL,
                 "The landmark should not be null!");
  const sm::kinematics::UncertainHomogeneousPoint & p_c = *pp_c;
  setPoint(T_v_c * p_c);
  setLandmarkId(kp.landmarkId());
}

}  // namespace aslam
