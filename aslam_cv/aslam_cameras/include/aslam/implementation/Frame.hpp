#include <aslam/cameras/CameraGeometryBase.hpp>
#include <sm/serialization_macros.hpp>

namespace aslam {

template<typename C>
Frame<C>::Frame() {
  _image.reset(new aslam::Image);
}
;

template<typename C>
Frame<C>::~Frame() {
}

template<typename C>
const typename Frame<C>::camera_geometry_t & Frame<C>::geometry() const {
  return *_geometry;
}

template<typename C>
boost::shared_ptr<C> Frame<C>::geometryPtr() const {
  return _geometry;
}

template<typename C>
void Frame<C>::setGeometryBase(
    const boost::shared_ptr<cameras::CameraGeometryBase> & geometry) {
  SM_ASSERT_TRUE(Exception, geometry.get() != NULL,
                 "Cannot set a null geometry pointer. Illegal!");
  boost::shared_ptr<C> cam = boost::dynamic_pointer_cast < C > (geometry);
  SM_ASSERT_TRUE(
      Exception,
      cam.get() != NULL,
      "Geometry pointer is the wrong type. Wanted: " << typeid(C).name()
          << ", got: " << typeid(*geometry.get()).name());
  _geometry = cam;
}

template<typename C>
void Frame<C>::setGeometry(
    const boost::shared_ptr<typename Frame<C>::camera_geometry_t> & cameraGeometryPtr) {
  _geometry = cameraGeometryPtr;
}

template<typename C>
void Frame<C>::clearGeometry() {
  _geometry.reset();
}

template<typename C>
void Frame<C>::clearKeypoints() {
  _keypoints.clear();
}

template<typename C>
const typename Frame<C>::keypoint_t & Frame<C>::keypoint(size_t i) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, _keypoints.size(),
                   "Keypoint index out of bounds");
  return _keypoints[i];
}

template<typename C>
typename Frame<C>::keypoint_t & Frame<C>::keypoint(size_t i) {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, _keypoints.size(),
                   "Keypoint index out of bounds");

  return _keypoints[i];
}

template<typename C>
typename Frame<C>::keypoint_t & Frame<C>::keypointChecked(size_t i) {
  SM_ASSERT_LT(IndexOutOfBoundsException, i, _keypoints.size(),
               "Keypoint index out of bounds");

  return _keypoints[i];
}

template<typename C>
const typename Frame<C>::keypoint_t & Frame<C>::keypointChecked(
    size_t i) const {
  SM_ASSERT_LT(IndexOutOfBoundsException, i, _keypoints.size(),
               "Keypoint index out of bounds");

  return _keypoints[i];
}

// The keypoint time. This will be something like: frame.time() + geometry.temporalOffset( keypoint(i) );
template<typename C>
Time Frame<C>::keypointTime(size_t i) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, _keypoints.size(),
                   "Keypoint index out of bounds");
  return _stamp + _geometry->temporalOffset(_keypoints[i].measurement());
}

template<typename C>
Time Frame<C>::keypointTime(const KeypointIdentifier & kid) const {
  return keypointTime(kid.keypointIndex);
}

// Keypoints can only be added, never removed. This is because data associations between
// frames are stored as indices.
template<typename C>
void Frame<C>::addKeypoint(const keypoint_t & keypoint) {
  _keypoints.push_back(keypoint);
}

template<typename C>
void Frame<C>::addKeypoints(const keypoint_list_t & keypoints) {
  _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
}

template<typename C>
typename Frame<C>::keypoint_t & Frame<C>::addKeypoint() {
  _keypoints.push_back(typename Frame<C>::keypoint_t());
  return _keypoints[_keypoints.size() - 1];

}

template<typename C>
KeypointBase & Frame<C>::addBaseKeypoint() {
  return addKeypoint();
}

template<typename C>
void Frame<C>::reserveKeypoints(size_t size) {
  _keypoints.reserve(size);
}

template<typename C>
boost::shared_ptr<cameras::CameraGeometryBase> Frame<C>::geometryBase() {
  return _geometry;
}

template<typename C>
boost::shared_ptr<const cameras::CameraGeometryBase> Frame<C>::geometryBase() const {
  return _geometry;
}

template<typename C>
const KeypointBase & Frame<C>::keypointBase(size_t i) const {
  return keypoint(i);
}

template<typename C>
KeypointBase & Frame<C>::keypointBase(size_t i) {
  return keypoint(i);
}

template<typename C>
size_t Frame<C>::numKeypoints() const {
  return _keypoints.size();
}

/// \brief compute the projection for a 4x1 homogeneous point
///        returns true if the projection was successful
template<typename C>
bool Frame<C>::computeProjection4(const Eigen::Vector4d & ph,
                                  Eigen::VectorXd & outReprojection) const {
  return _geometry->homogeneousToKeypoint(ph, outReprojection);
}

/// \brief compute the projection for a 3x1 point
///        returns true if the projection was successful
template<typename C>
bool Frame<C>::computeProjection3(const Eigen::Vector3d & ph,
                                  Eigen::VectorXd & outReprojection) const {
  return _geometry->euclideanToKeypoint(ph, outReprojection);
  return _geometry->euclideanToKeypoint(ph, outReprojection);;
}

/// \brief compute the projection for an uncertain homogeneous point
///        returns true if the projection was successful
template<typename C>
bool Frame<C>::computeProjectionUhp(
    const sm::kinematics::UncertainHomogeneousPoint & ph,
    Eigen::VectorXd & outReprojection) const {
  return _geometry->homogeneousToKeypoint(ph.toHomogeneous(), outReprojection);;
}

/// \brief compute the reprojection error for this keypoint from a Euclidean point
template<typename C>
bool Frame<C>::computeReprojectionError3(size_t i, const Eigen::Vector3d & p,
                                         double & outReprojectionError) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, numKeypoints(),
                   "Index out of bounds");
  measurement_t y;
  bool success = _geometry->euclideanToKeypoint(p, y);
  if (success) {
    const keypoint_t & k = _keypoints[i];
    y -= k.y();
    outReprojectionError = y.dot(k.invR() * y);
  }
  return success;
}

/// \brief compute the reprojection error for this keypoint from a Homogeneous point
template<typename C>
bool Frame<C>::computeReprojectionError4(size_t i, const Eigen::Vector4d & p,
                                         double & outReprojectionError) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, numKeypoints(),
                   "Index out of bounds");
  measurement_t y;
  bool success = _geometry->homogeneousToKeypoint(p, y);
  if (success) {
    const keypoint_t & k = _keypoints[i];
    y -= k.y();
    outReprojectionError = y.dot(k.invR() * y);
  }
  return success;
}

/// \brief compute the reprojection error for this keypoint from an uncertain Homogeneous point
template<typename C>
bool Frame<C>::computeReprojectionErrorUhp(
    size_t i, const sm::kinematics::UncertainHomogeneousPoint & p,
    double & outReprojectionError) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, numKeypoints(),
                   "Index out of bounds");
  measurement_t y;
  typename camera_geometry_t::jacobian_homogeneous_t J;
  typename camera_geometry_t::covariance_t invR;
  bool success = _geometry->homogeneousToKeypoint(p.toHomogeneous(), y, J);
  if (success) {
    const keypoint_t & k = _keypoints[i];
    y -= k.y();

    invR = (k.invR().inverse() + J * p.U4() * J.transpose()).inverse();

    outReprojectionError = y.dot(invR * y);
  }
  return success;

}

///////////////////////////////////////////////////
// A Generic interface for camera systems
///////////////////////////////////////////////////

/// \brief compute the reprojection error for this keypoint from a Euclidean point
template<typename C>
bool Frame<C>::computeReprojectionError3(const KeypointIdentifier & kid,
                                         const Eigen::Vector3d & p,
                                         double & outReprojectionError) const {
  return computeReprojectionError3(kid.keypointIndex, p, outReprojectionError);
}

/// \brief compute the reprojection error for this keypoint from a Homogeneous point
template<typename C>
bool Frame<C>::computeReprojectionError4(const KeypointIdentifier & kid,
                                         const Eigen::Vector4d & p,
                                         double & outReprojectionError) const {
  return computeReprojectionError4(kid.keypointIndex, p, outReprojectionError);
}

/// \brief compute the reprojection error for this keypoint from an uncertain Homogeneous point
template<typename C>
bool Frame<C>::computeReprojectionErrorUhp(
    const KeypointIdentifier & kid,
    const sm::kinematics::UncertainHomogeneousPoint & p,
    double & outReprojectionError) const {
  return computeReprojectionErrorUhp(kid.keypointIndex, p, outReprojectionError);
}

/// \brief get the keypoint associated with this identifier.
template<typename C>
void Frame<C>::getKeypoint(const KeypointIdentifier & kid,
                           keypoint_t & outKeypoint) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, kid.keypointIndex, numKeypoints(),
                   "Index out of bounds");
  outKeypoint = keypoint(kid.keypointIndex);
}

/// \brief get a const ref to the list of keypoints.
template<typename C>
const typename Frame<C>::keypoint_list_t& Frame<C>::getKeypoints() const {
  return _keypoints;
}

template<typename C>
bool Frame<C>::isBinaryEqual(const FrameBase & rhs) const {
  const Frame<C> * F = dynamic_cast<const Frame<C> *>(&rhs);
  if (!F) {
    return false;
  }

  return isBinaryEqual(*F);
}

template<typename C>
bool Frame<C>::isBinaryEqual(const Frame<C> & rhs) const {

  bool isEqual = true;

  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _image)
      && SM_CHECKMEMBERSSAME(rhs, _stamp) && SM_CHECKMEMBERSSAME(rhs, _id)
      && SM_CHECKSAME(_keypoints.size(), rhs._keypoints.size());

  isEqual = isEqual && SM_CHECKSAME(_geometry, rhs._geometry);

  if (isEqual) {
    for (unsigned i = 0; isEqual && i < _keypoints.size(); ++i) {
      isEqual = SM_CHECKSAME(_keypoints[i], rhs._keypoints[i]);
    }
  }

  return isEqual;
}

// Support for unit testing.
template<typename C>
void Frame<C>::setRandom() {
  // Random image.
  _image->getOctaveMutable(0).create(std::max(8, rand() & 0x111),
                                     std::max(8, rand() & 0x111), CV_8UC1);

  _stamp = Time::now();

  _id = FrameId(rand());
  // Not random but...whatever.
  _geometry.reset(new camera_geometry_t(camera_geometry_t::getTestGeometry()));

  // some small random number of keypoints.
  int nkeypoints = std::max(8, rand() & 0x1111);
  _keypoints.resize(nkeypoints);
  for (int i = 0; i < nkeypoints; ++i) {
    _keypoints[i].setRandom();
  }

}

/// \brief get the landmark for keypoint k. Return true on success
template<typename C>
bool Frame<C>::getLandmark(size_t k, Eigen::Vector4d & outLandmark) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, k, numKeypoints(),
                   "Index out of bounds");

  const sm::kinematics::UncertainHomogeneousPoint * p = _keypoints[k]
      .landmarkPtr();

  if (p) {
    outLandmark = p->toHomogeneous();
  }

  return p;
}

/// \brief get the landmark for keypoint k. Return true on success
template<typename C>
bool Frame<C>::getLandmark(const KeypointIdentifier kid,
                           Eigen::Vector4d & outLandmark) const {
  return getLandmark(kid.keypointIndex, outLandmark);
}

/// \brief get the back projection for keypoint i
template<typename C>
void Frame<C>::getBackProjection(size_t i,
                                 BackProjection & outBackProjection) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, numKeypoints(),
                   "Index out of bounds");

  outBackProjection.viewOrigin.setZero();

  const keypoint_t & k = _keypoints[i];
  if (k.isBackProjectionSet()) {
    outBackProjection.ray = k.backProjection()->mean();
  } else {
    Eigen::Vector3d bp(0.0, 0.0, 0.0);
    geometry().keypointToEuclidean(k.y(), bp);

    // \todo. Set the back projection here?

    outBackProjection.ray = bp;
    outBackProjection.ray.normalize();

  }

}

/// \brief get the back projection for keypoint kid
template<typename C>
void Frame<C>::getBackProjection(const KeypointIdentifier & kid,
                                 BackProjection & outBackProjection) const {
  getBackProjection(kid.keypointIndex, outBackProjection);
}

/// \brief get the back projection for keypoint i
template<typename C>
void Frame<C>::getUncertainBackProjection(
    size_t i, UncertainBackProjection & outBackProjection) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, i, numKeypoints(),
                   "Index out of bounds");

  outBackProjection.viewOrigin.setZero();

  const keypoint_t & k = _keypoints[i];
  if (k.isBackProjectionSet()) {
    outBackProjection = *k.backProjection();
  } else {
    Eigen::Vector3d bp(0.0, 0.0, 0.0);
    typename camera_geometry_t::inverse_jacobian_t Jb;
    geometry().keypointToEuclidean(k.y(), bp, Jb);

    Eigen::Matrix3d P = Jb * k.invR().inverse() * Jb.transpose();
    outBackProjection.ray.setMean(bp);
    outBackProjection.ray.setCovariance(P);
    outBackProjection.ray.normalize();

  }

}

/// \brief get the back projection for keypoint kid
template<typename C>
void Frame<C>::getUncertainBackProjection(
    const KeypointIdentifier & kid,
    UncertainBackProjection & outBackProjection) const {
  getUncertainBackProjection(kid.keypointIndex, outBackProjection);
}

/// \brief compute and store all back projections.
template<typename C>
void Frame<C>::computeAllBackProjections(bool doBackProjectionUncertainty) {
  if (doBackProjectionUncertainty) {
    doBackProjectionWithUncertainty(*this);
  } else {
    doBackProjection(*this);
  }
}

template<typename C>
template<class Archive>
void Frame<C>::save(Archive & ar, const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(FrameBase);
  ar << BOOST_SERIALIZATION_NVP(_image);
  ar << BOOST_SERIALIZATION_NVP(_stamp);
  ar << BOOST_SERIALIZATION_NVP(_id);
  ar << BOOST_SERIALIZATION_NVP(_geometry);
  ar << BOOST_SERIALIZATION_NVP(_keypoints);
}

template<typename C>
template<class Archive>
void Frame<C>::load(Archive & ar, const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");
  //boost::serialization::void_cast_register< aslam::Frame< C >, aslam::FrameBase>(static_cast<aslam::Frame< C > *>(NULL), static_cast< FrameBase * >(NULL) );
  if (version >= 1) {
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(FrameBase);
  }

  if (version >= 2) {
    ar >> BOOST_SERIALIZATION_NVP(_image);
  } else {    //for older versions only deserialize the cv::Mat
    cv::Mat img;
    ar >> BOOST_SERIALIZATION_NVP(img);
    _image.reset(new aslam::Image);
    _image->getOctaveMutable(0) = img;
  }

  ar >> BOOST_SERIALIZATION_NVP(_stamp);
  ar >> BOOST_SERIALIZATION_NVP(_id);
  ar >> BOOST_SERIALIZATION_NVP(_geometry);
  ar >> BOOST_SERIALIZATION_NVP(_keypoints);
}

template<typename C>
void Frame<C>::addBaseKeypoints(const KeypointBase * kp, size_t n) {
  SM_ASSERT_TRUE(Exception, kp != NULL, "Cannot add null keypoints");
  if (n > 0) {
    const keypoint_t * k = dynamic_cast<const keypoint_t *>(kp);
    SM_ASSERT_TRUE(
        Exception,
        k != NULL,
        "Unable to cast the pointer to the underlying type of the frame: "
            << typeid(keypoint_t).name());
    _keypoints.insert(_keypoints.end(), k, k + n);
  }
}

template<typename C>
void Frame<C>::addBaseKeypoint(const KeypointBase * kp) {
  SM_ASSERT_TRUE(Exception, kp != NULL, "Cannot add null keypoints");
  const keypoint_t * k = dynamic_cast<const keypoint_t *>(kp);
  SM_ASSERT_TRUE(
      Exception,
      k != NULL,
      "Unable to cast the pointer to the underlying type of the frame: "
          << typeid(keypoint_t).name());
  addKeypoint(*k);
}

}  // namespace aslam
