#include <aslam/KeypointBase.hpp>

namespace aslam {

KeypointBase::KeypointBase()
    : _landmarkEnabled(true) {
}
KeypointBase::~KeypointBase() {
}

bool KeypointBase::isLandmarkEnabled() const {
  return _landmarkEnabled;
}

void KeypointBase::disableLandmark() {
  _landmarkEnabled = false;
}

void KeypointBase::enableLandmark() {
  _landmarkEnabled = true;
}

}  // namespace aslam
