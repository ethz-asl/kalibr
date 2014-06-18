namespace aslam {

template<typename CAMERA_T>
NullUndistorter<CAMERA_T>::NullUndistorter() {
  _idealGeometry.reset();
}

template<typename CAMERA_T>
NullUndistorter<CAMERA_T>::NullUndistorter(
    const sm::PropertyTree& undistorterConfig,
    const sm::PropertyTree& cameraConfig) {
  boost::shared_ptr<distorted_geometry_t> distortedGeometry(
      new distorted_geometry_t(cameraConfig));
  int interpolation = undistorterConfig.getInt("interpolationType",
                                               cv::INTER_NEAREST);
  double scale = undistorterConfig.getDouble("scale");
  bool makeCopy = undistorterConfig.getBool("makeCopy");
  init(distortedGeometry, interpolation, scale, makeCopy);
}

template<typename CAMERA_T>
NullUndistorter<CAMERA_T>::NullUndistorter(const sm::PropertyTree& cameraConfig,
                                           int interpolation, double scale,
                                           bool makeCopy) {
  boost::shared_ptr<distorted_geometry_t> distortedGeometry(
      new distorted_geometry_t(cameraConfig));
  init(distortedGeometry, interpolation, scale, makeCopy);
}

template<typename CAMERA_T>
NullUndistorter<CAMERA_T>::NullUndistorter(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    int interpolation, double scale, bool makeCopy) {
  init(distortedGeometry, interpolation, scale, makeCopy);
}

template<typename CAMERA_T>
NullUndistorter<CAMERA_T>::NullUndistorter(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    const sm::PropertyTree & config) {
  int interpolation = config.getInt("interpolationType", cv::INTER_NEAREST);
  double scale = config.getDouble("scale");
  bool makeCopy = config.getBool("makeCopy");
  init(distortedGeometry, interpolation, scale, makeCopy);
}

template<typename CAMERA_T>
NullUndistorter<CAMERA_T>::~NullUndistorter() {
}

template<typename CAMERA_T>
void NullUndistorter<CAMERA_T>::init(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    int interpolation, double scale, bool makeCopy) {
  _interpolation = interpolation;
  _scale = scale;
  _makeCopy = makeCopy;
  setIdealGeometry(distortedGeometry);
}

template<typename CAMERA_T>
void NullUndistorter<CAMERA_T>::constructUndistortedFrame(
    const cv::Mat & image, frame_t & outFrame) const {
  SM_ASSERT_TRUE(std::runtime_error, _idealGeometry,
                 "Camera has not yet been set.")

  if (_scale != 1.0 || _makeCopy) {
    cv::Mat undistImage;
    undistortImage(image, undistImage);
    outFrame.setImage(undistImage);
    outFrame.setGeometry(_idealGeometry);
  } else {
    outFrame.setImage(image);
    outFrame.setGeometry(_idealGeometry);
  }
}

template<typename CAMERA_T>
void NullUndistorter<CAMERA_T>::undistortImage(const cv::Mat & inImage,
                                               cv::Mat & outImage) const {
  if (_scale != 1.0) {
    cv::resize(inImage, outImage, cv::Size(), _scale, _scale, _interpolation);
  } else if (_makeCopy) {
    outImage = inImage.clone();
  } else {
    outImage = inImage;
  }
}

template<typename CAMERA_T>
boost::shared_ptr<typename NullUndistorter<CAMERA_T>::ideal_geometry_t> NullUndistorter<
    CAMERA_T>::idealGeometry() const {
  return _idealGeometry;
}

template<typename CAMERA_T>
boost::shared_ptr<typename NullUndistorter<CAMERA_T>::ideal_geometry_t> NullUndistorter<
    CAMERA_T>::distortedGeometry() const {
  return _idealGeometry;
}

template<typename CAMERA_T>
void NullUndistorter<CAMERA_T>::setIdealGeometry(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry) {
  // set new idealGeometry
  _idealGeometry = boost::shared_ptr < ideal_geometry_t
      > (new ideal_geometry_t(distortedGeometry->projection(),
                              distortedGeometry->shutter(),
                              distortedGeometry->mask()));

  // resize intrinstics
  if (_scale != 1.0) {
    _idealGeometry->projection().resizeIntrinsics(_scale);
  }
}

template<typename CAMERA_T>
boost::shared_ptr<FrameBase> NullUndistorter<CAMERA_T>::buildFrame(
    cv::Mat & inImage) {

  boost::shared_ptr < frame_t > frame(new frame_t());
  constructUndistortedFrame(inImage, *frame);

  return frame;

}

}  // namespace aslam
