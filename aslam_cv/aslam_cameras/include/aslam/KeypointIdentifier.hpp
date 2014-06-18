#ifndef ASLAM_KEYPOINT_IDENTIFIER_HPP
#define ASLAM_KEYPOINT_IDENTIFIER_HPP

#include <aslam/frontend_ids.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/random.hpp>
#include <iostream>

namespace aslam {

struct KeypointIdentifier {
  KeypointIdentifier(MultiFrameId fi = MultiFrameId(), size_t ci = 0,
                     size_t ki = 0)
      : frameId(fi),
        cameraIndex(ci),
        keypointIndex(ki) {
  }

  MultiFrameId frameId;
  size_t cameraIndex;
  size_t keypointIndex;

  MultiFrameId getFrameId() {
    return frameId;
  }
  void setFrameId(MultiFrameId fid) {
    frameId = fid;
  }

  void setRandom() {
    frameId.setRandom();
    cameraIndex = sm::random::randLUi(0, 200);
    keypointIndex = sm::random::randLUi(0, 200);
  }

  bool isBinaryEqual(const KeypointIdentifier & rhs) const {
    return frameId == rhs.frameId && cameraIndex == rhs.cameraIndex
        && keypointIndex == rhs.keypointIndex;
  }

  bool operator==(const KeypointIdentifier & rhs) const {
    return isBinaryEqual(rhs);
  }

  bool operator<(const KeypointIdentifier & rhs) const {

    if (frameId == rhs.frameId) {
      if (cameraIndex == rhs.cameraIndex) {
        return keypointIndex < rhs.keypointIndex;
      } else {
        return cameraIndex < rhs.cameraIndex;
      }
    }
    return frameId < rhs.frameId;
  }

  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int) CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");

    ar >> BOOST_SERIALIZATION_NVP(frameId);
    ar >> BOOST_SERIALIZATION_NVP(cameraIndex);
    ar >> BOOST_SERIALIZATION_NVP(keypointIndex);
  }

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(frameId);
    ar << BOOST_SERIALIZATION_NVP(cameraIndex);
    ar << BOOST_SERIALIZATION_NVP(keypointIndex);

  }

};

}  // namespace aslam

inline std::ostream & operator<<(std::ostream & os,
                                 const aslam::KeypointIdentifier & kid) {
  os << "[" << kid.frameId << ", " << kid.cameraIndex << ", "
      << kid.keypointIndex << "]";
  return os;
}

SM_BOOST_CLASS_VERSION (aslam::KeypointIdentifier);

#endif /* ASLAM_KEYPOINT_IDENTIFIER_HPP */
