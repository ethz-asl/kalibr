namespace aslam {

template<int D>
Keypoint<D>::Keypoint()
    : KeypointBase(),
      _measurement(measurement_t::Zero()),
      _inverseMeasurementCovariance(inverse_covariance_t::Identity()),
      _landmark(NULL),
      _backProjection(NULL),
      _octave(0),
      _traceId(0) {

}

/// \brief the copy constructor makes a deep copy
template<int D>
Keypoint<D>::Keypoint(const Keypoint & rhs)
    : KeypointBase(),
      _measurement(rhs._measurement),
      _inverseMeasurementCovariance(rhs._inverseMeasurementCovariance),
      _landmarkId(rhs._landmarkId),
      _landmark(NULL),
      _descriptor(rhs._descriptor),
      _backProjection(NULL),
      _octave(rhs._octave),
      _traceId(rhs._traceId)

{
  if (rhs._landmark) {
    _landmark = new sm::kinematics::UncertainHomogeneousPoint(*rhs._landmark);
  }

  if (rhs._backProjection) {
    _backProjection = new sm::kinematics::UncertainVector3(
        *rhs._backProjection);
  }

}

template<int D>
template<typename DERIVED>
double Keypoint<D>::mahalonibisDistance(
    const Eigen::MatrixBase<DERIVED> & y) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, D);
  measurement_t e = y.derived().template head<D>() - _measurement;

  return e.dot(_inverseMeasurementCovariance * e);
}

template<int D>
double Keypoint<D>::vsMahalonibisDistance(const Eigen::VectorXd & y) const {
  return mahalonibisDistance(y);
}

template<int D>
void Keypoint<D>::freeAll() {
  if (_landmark) {
    delete _landmark;
    _landmark = NULL;
  }

  if (_backProjection) {
    delete _backProjection;
    _backProjection = NULL;
  }

}

/// \brief the assignment operator makes a deep copy
template<int D>
Keypoint<D> & Keypoint<D>::operator=(const Keypoint & rhs) {
  Keypoint copy(rhs);
  this->swap(copy);
  return *this;
}

template<int D>
Keypoint<D>::~Keypoint() {
  freeAll();
}

/// \brief the image measurement
template<int D>
const typename Keypoint<D>::measurement_t & Keypoint<D>::measurement() const {
  return _measurement;
}

/// \brief the image measurement
template<int D>
const typename Keypoint<D>::measurement_t & Keypoint<D>::y() const {
  return _measurement;
}

/// \brief set the image measurement
template<int D>
void Keypoint<D>::setMeasurement(const measurement_t & m) {
  _measurement = m;
}

/// \brief the octave this keypoint was detected in
template<int D>
int Keypoint<D>::octave() const {
  return _octave;
}

/// \brief set the octave this keypoint was detected in
template<int D>
void Keypoint<D>::setOctave(const int octave) {
  _octave = octave;
}

/// \brief get the image measurement inverse covariance
template<int D>
const typename Keypoint<D>::inverse_covariance_t & Keypoint<D>::inverseMeasurementCovariance() const {
  return _inverseMeasurementCovariance;
}

/// \brief get the image measurement inverse covariance
template<int D>
const typename Keypoint<D>::inverse_covariance_t & Keypoint<D>::invR() const {
  return _inverseMeasurementCovariance;
}

/// \brief set the image measurement inverse covariance
template<int D>
void Keypoint<D>::setInverseMeasurementCovariance(
    const inverse_covariance_t & invR) {
  _inverseMeasurementCovariance = invR;
}

template<int D>
Eigen::VectorXd Keypoint<D>::vsMeasurement() const {
  return _measurement;
}

template<int D>
void Keypoint<D>::vsSetMeasurement(const Eigen::VectorXd & m) {
  SM_ASSERT_EQ(Exception, m.size(), D, "Invalid vector size");
  _measurement = m;
}

template<int D>
Eigen::MatrixXd Keypoint<D>::vsInverseMeasurementCovariance() const {
  return _inverseMeasurementCovariance;
}

template<int D>
void Keypoint<D>::vsSetInverseMeasurementCovariance(
    const Eigen::MatrixXd & invR) {
  SM_ASSERT_EQ(Exception, invR.rows(), D, "Invalid matrix size");
  SM_ASSERT_EQ(Exception, invR.cols(), D, "Invalid matrix size");
  _inverseMeasurementCovariance = invR;
}

/// \brief get the keypoint descriptor
template<int D>
const DescriptorBase * Keypoint<D>::descriptor() const {
  return _descriptor.get();
}

/// \brief get the keypoint descriptor
template<int D>
boost::shared_ptr<DescriptorBase> Keypoint<D>::descriptorPtr() const {
  return _descriptor;
}

/// \brief set the keypoint descriptor
template<int D>
void Keypoint<D>::setDescriptor(const DescriptorBase & descriptor) {
  _descriptor.reset(descriptor.clone());
}

template<int D>
void Keypoint<D>::setDescriptorRawPtr(DescriptorBase * descriptor) {
  _descriptor.reset(descriptor);
}

template<int D>
void Keypoint<D>::setDescriptorPtr(
    boost::shared_ptr<DescriptorBase> descriptor) {

  _descriptor = descriptor;
}

/// \brief get the landmark
template<int D>
const sm::kinematics::UncertainHomogeneousPoint * Keypoint<D>::landmarkPtr() const {
  return _landmark;
}

/// \brief get the landmark
template<int D>
const sm::kinematics::UncertainHomogeneousPoint & Keypoint<D>::landmark() const {
  SM_ASSERT_TRUE_DBG(Exception, _landmark != NULL,
                     "Getting a null landmark is bad, okay?");
  return *_landmark;
}

/// \brief set the landmark
template<int D>
void Keypoint<D>::setLandmark(
    const sm::kinematics::UncertainHomogeneousPoint & landmark) {
  if (!_landmark) {
    _landmark = new sm::kinematics::UncertainHomogeneousPoint(landmark);
  } else {
    *_landmark = landmark;
  }

}

/// \brief get the landmark id
template<int D>
const LandmarkId & Keypoint<D>::landmarkId() const {
  return _landmarkId;
}

/// \brief set the landmark id
template<int D>
void Keypoint<D>::setLandmarkId(const LandmarkId & landmarkId) {
  _landmarkId = landmarkId;
}

/// \brief is the landmark initialized
template<int D>
bool Keypoint<D>::isLandmarkInitialized() const {
  return _landmark;
}

/// \brief reset and clear the landmark.
template<int D>
void Keypoint<D>::clearLandmark() {
  if (_landmark) {
    delete _landmark;
    _landmark = NULL;
  }
  _landmarkId.clear();
}

/// \brief set the back projection
template<int D>
void Keypoint<D>::setBackProjection(
    const sm::kinematics::UncertainVector3 & v) {
  if (_backProjection) {
    delete _backProjection;
  }

  _backProjection = new sm::kinematics::UncertainVector3(v);
}

template<int D>
void Keypoint<D>::setBackProjectionVector(const Eigen::Vector3d & v) {
  if (_backProjection) {
    delete _backProjection;
  }

  _backProjection = new sm::kinematics::UncertainVector3(v);
}

/// \brief get the back projection pointer
template<int D>
const sm::kinematics::UncertainVector3 * Keypoint<D>::backProjection() const {
  return _backProjection;
}

/// \brief is the back projection initialized?
template<int D>
bool Keypoint<D>::isBackProjectionSet() const {
  return _backProjection;
}

/// \brief clear the back projeciton
template<int D>
void Keypoint<D>::clearBackProjection() {
  if (_backProjection) {
    delete _backProjection;
    _backProjection = NULL;
  }
}

template<int D>
void Keypoint<D>::swap(Keypoint & rhs) {
  if (this != &rhs) {
    _measurement.swap(rhs._measurement);
    _inverseMeasurementCovariance.swap(rhs._inverseMeasurementCovariance);
    _landmarkId.swap(rhs._landmarkId);
    std::swap(_landmark, rhs._landmark);
    _descriptor.swap(rhs._descriptor);
    std::swap(_backProjection, rhs._backProjection);
  }
}

template<int D>
void Keypoint<D>::setTraceId(const size_t traceId) {
  _traceId = traceId;
}

template<int D>
size_t Keypoint<D>::getTraceId() {
  return _traceId;
}

template<int D>
bool Keypoint<D>::isBinaryEqual(const Keypoint<D> & rhs) const {
  bool isEqual = true;
  // ar >> BOOST_SERIALIZATION_NVP(_measurement);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _measurement);
  // ar >> BOOST_SERIALIZATION_NVP(_inverseMeasurementCovariance);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _inverseMeasurementCovariance);
  // ar >> BOOST_SERIALIZATION_NVP(_landmarkId);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _landmarkId);
  // ar >> BOOST_SERIALIZATION_NVP(_landmark);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _landmark);
  // ar >> BOOST_SERIALIZATION_NVP(_descriptor);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _descriptor);
  // ar >> BOOST_SERIALIZATION_NVP(_backProjection);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _backProjection);
  // ar >> BOOST_SERIALIZATION_NVP(_viewOrigin);
  isEqual = isEqual && SM_CHECKMEMBERSSAME(rhs, _octave);
  return isEqual;

}

template<int D>
void Keypoint<D>::setRandom() {
  freeAll();
  /// \brief The keypoint measurement
  _measurement.setRandom();

  /// \brief The inverse covariance measurement
  inverse_covariance_t U;
  U.setRandom();
  _inverseMeasurementCovariance = U.transpose() * U
      + inverse_covariance_t::Identity();

  /// \brief the landmark 
  _landmarkId.setRandom();

  /// \brief the landmark.
  if (sm::random::rand() > 0.5) {
    _landmark = new sm::kinematics::UncertainHomogeneousPoint;
    _landmark->setRandom();
  }

  /// \brief the keypoint descriptor
  /// \todo add a descriptor
  // boost::shared_ptr<DescriptorBase> _descriptor;

  /// \brief the back projection, a ray in space originating from the view origin.
  if (sm::random::rand() > 0.5) {
    _backProjection = new sm::kinematics::UncertainVector3;
    _backProjection->setRandom();
  }

  _octave = 0;
}

}
