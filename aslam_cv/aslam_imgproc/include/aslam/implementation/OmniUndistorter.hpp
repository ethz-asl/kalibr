namespace aslam {

template<typename MASK_T>
OmniUndistorter<MASK_T>::OmniUndistorter() {
  _idealGeometry.reset();
  _idealPinholeGeometry.reset();
}

template<typename MASK_T>
OmniUndistorter<MASK_T>::OmniUndistorter(
    const sm::PropertyTree& undistorterConfig,
    const sm::PropertyTree& cameraConfig) {
  boost::shared_ptr<distorted_geometry_t> distortedGeometry(
      new distorted_geometry_t(cameraConfig));
  int interpolation = undistorterConfig.getInt("interpolationType",
                                               cv::INTER_NEAREST);
  double alpha = undistorterConfig.getDouble("alpha");
  double scale = undistorterConfig.getDouble("scale");
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename MASK_T>
OmniUndistorter<MASK_T>::OmniUndistorter(const sm::PropertyTree& cameraConfig,
                                         int interpolation, double alpha,
                                         double scale) {
  boost::shared_ptr<distorted_geometry_t> distortedGeometry(
      new distorted_geometry_t(cameraConfig));
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename MASK_T>
OmniUndistorter<MASK_T>::OmniUndistorter(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    int interpolation, double alpha, double scale) {
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename MASK_T>
OmniUndistorter<MASK_T>::OmniUndistorter(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    const sm::PropertyTree & config) {
  int interpolation = config.getInt("interpolationType", cv::INTER_NEAREST);
  double alpha = config.getDouble("alpha");
  double scale = config.getDouble("scale");
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename MASK_T>
OmniUndistorter<MASK_T>::~OmniUndistorter() {
}

template<typename MASK_T>
void OmniUndistorter<MASK_T>::init(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    int interpolation, double alpha, double scale) {
  _interpolation = interpolation;
  setIdealGeometry(distortedGeometry, alpha, scale);
  setIdealPinholeGeometry(_idealGeometry);
}

template<typename MASK_T>
void OmniUndistorter<MASK_T>::constructUndistortedFrame(
    const cv::Mat & image, frame_t & outFrame) const {
  SM_ASSERT_TRUE(std::runtime_error, _idealGeometry,
                 "Camera has not yet been set.")
  cv::Mat undistImage;
  undistortImage(image, undistImage);
  outFrame.setImage(undistImage);
  outFrame.setGeometry(_idealGeometry);
}

template<typename MASK_T>
void OmniUndistorter<MASK_T>::undistortImage(const cv::Mat & inImage,
                                             cv::Mat & outImage) const {
  // see: http://opencv.willowgarage.com/documentation/cpp/imgproc_geometric_image_transformations.html#remap
  cv::remap(inImage, outImage, mapX, mapY, _interpolation);
}

//intialize a map to convert an undistorted omni to a pinhole image (with alpha=1, scale=1.0)
template<typename CAMERA_T>
void initUndistOmniToPinholeMap(boost::shared_ptr<CAMERA_T> camera_geometry,
                                const Eigen::Matrix3d &_pinholeCameraMatrix, cv::Size size,
                                cv::OutputArray _map1, cv::OutputArray _map2) {
  using namespace cv;

  //prepare the outputs data structures
  _map1.create(size, CV_16SC2);
  _map2.create(size, CV_16UC1);
  Mat map1 = _map1.getMat(),
      map2 = _map2.getMat();

  //invert
  Eigen::Matrix3d invR = _pinholeCameraMatrix.inverse();

  for (int i = 0; i < size.height; i++) {
    float* m1f = (float*) (map1.data + map1.step * i);
    float* m2f = (float*) (map2.data + map2.step * i);
    short* m1 = (short*) m1f;
    ushort* m2 = (ushort*) m2f;

    double _x = i * invR(0, 1) + invR(0, 2),
           _y = i * invR(1, 1) + invR(1, 2),
           _w = i * invR(2, 1) + invR(2, 2);

    for (int j = 0; j < size.width; j++,
                                    _x += invR(0, 0),
                                    _y += invR(1, 0),
                                    _w += invR(2, 0))
    {
      //apply original distortion
      Eigen::Vector3d point(_x, _y, _w);
      Eigen::Vector2d keypoint;
      camera_geometry->euclideanToKeypoint(point, keypoint);

      //store in output format
      int iu = saturate_cast<int>(keypoint[0] * INTER_TAB_SIZE);
      int iv = saturate_cast<int>(keypoint[1] * INTER_TAB_SIZE);
      m1[j * 2] = (short) (iu >> INTER_BITS);
      m1[j * 2 + 1] = (short) (iv >> INTER_BITS);
      m2[j] = (ushort) ((iv & (INTER_TAB_SIZE - 1)) * INTER_TAB_SIZE
          + (iu & (INTER_TAB_SIZE - 1)));
    }
  }
}

template<typename MASK_T>
void OmniUndistorter<MASK_T>::undistortImageToPinhole(const cv::Mat & inImage,
                                                      cv::Mat & outImage) const {
  //first undistort image
  cv::Mat undistImage;
  undistortImage(inImage, undistImage);

  //convert to pinhole image
  cv::remap(undistImage, outImage, mapX_pinhole, mapY_pinhole, _interpolation);
}

template<typename MASK_T>
boost::shared_ptr<typename OmniUndistorter<MASK_T>::ideal_geometry_t> OmniUndistorter<
    MASK_T>::idealGeometry() const {
  return _idealGeometry;
}

template<typename MASK_T>
boost::shared_ptr<typename OmniUndistorter<MASK_T>::ideal_pinhole_geometry_t> OmniUndistorter<
    MASK_T>::idealPinholeGeometry() const {
  return _idealPinholeGeometry;
}

template<typename MASK_T>
boost::shared_ptr<typename OmniUndistorter<MASK_T>::distorted_geometry_t> OmniUndistorter<
    MASK_T>::distortedGeometry() const {
  return _distortedGeometry;
}

template<typename MASK_T>
void OmniUndistorter<MASK_T>::setIdealGeometry(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry, double alpha,
    double scale) {
  _distortedGeometry = distortedGeometry;
  Eigen::Matrix3d distortedCameraMatrix = distortedGeometry->projection()
      .getCameraMatrix();

  // get distortion: (k1, k2, p1, p2)'
  Eigen::MatrixXd distortionParameters;
  distortedGeometry->projection().distortion().getParameters(
      distortionParameters);

  // convert to CV
  cv::Mat distortionParametersCV;
  cv::eigen2cv(distortionParameters, distortionParametersCV);

  int width = distortedGeometry->projection().ru();
  int height = distortedGeometry->projection().rv();
  int newWidth = (int) width * scale;
  int newHeight = (int) height * scale;

  cv::Mat distortedCameraMatrixCV;
  cv::eigen2cv(distortedCameraMatrix, distortedCameraMatrixCV);

  // compute the optimal new camera matrix based on the free scaling parameter
  // see: http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#getOptimalNewCameraMatrix
  cv::Mat idealCameraMatrixCV = cv::getOptimalNewCameraMatrix(
      distortedCameraMatrixCV, distortionParametersCV, cv::Size(width, height),
      alpha, cv::Size(newWidth, newHeight));
  Eigen::Matrix3d idealCameraMatrix;
  cv::cv2eigen(idealCameraMatrixCV, idealCameraMatrix);

  // create idealProjection
  typename ideal_geometry_t::projection_t idealProjection(
      distortedGeometry->projection().xi(), idealCameraMatrix(0, 0),
      idealCameraMatrix(1, 1), idealCameraMatrix(0, 2), idealCameraMatrix(1, 2),
      newWidth, newHeight,
      typename ideal_geometry_t::projection_t::distortion_t());

  // set new idealGeometry
  _idealGeometry = boost::shared_ptr < ideal_geometry_t
      > (new ideal_geometry_t(idealProjection, distortedGeometry->shutter(),
                              distortedGeometry->mask()));

  // compute the undistortion and rectification transformation map
  // see: http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#initUndistortRectifyMap
  cv::initUndistortRectifyMap(distortedCameraMatrixCV, distortionParametersCV,
                              cv::Mat(), idealCameraMatrixCV,
                              cv::Size(newWidth, newHeight), CV_16SC2, mapX,
                              mapY);
}

template<typename MASK_T>
void OmniUndistorter<MASK_T>::setIdealPinholeGeometry(boost::shared_ptr<ideal_geometry_t> idealGeometry) {
  //calculate the new size
  size_t width = idealGeometry->projection().ru();
  size_t height = idealGeometry->projection().rv();

  //get surrounding box
  Eigen::Vector2d keypoint_top_left(0,0),
                  keypoint_bottom_right(width,height);
  Eigen::Vector3d point_top_left,
                  point_bottom_right;

  idealGeometry->keypointToEuclidean(keypoint_top_left, point_top_left);
  idealGeometry->keypointToEuclidean(keypoint_bottom_right, point_bottom_right);

  point_top_left = point_top_left / point_top_left[2];
  point_bottom_right = point_bottom_right / point_bottom_right[2];

  //fit the focal such that the resolution stays the same
  Eigen::Matrix3d newCameraMatrix = Eigen::Matrix3d::Zero();
  newCameraMatrix(0,0) = width/(point_bottom_right[0]-point_top_left[0]);
  newCameraMatrix(1,1) = height/(point_bottom_right[1]-point_top_left[1]);
  newCameraMatrix(0,2) = (width-1)/2.0;
  newCameraMatrix(1,2) = (height-1)/2.0;
  newCameraMatrix(2,2) = 1.0;

  // create omni to pinhole mapping
  cv::Size newsize(width, height);
  initUndistOmniToPinholeMap(idealGeometry,
                             newCameraMatrix,
                             newsize,
                             mapX_pinhole, mapY_pinhole);

  // create pinhole camera
  typename ideal_pinhole_geometry_t::projection_t idealPinholeProjection(
      newCameraMatrix(0, 0),
      newCameraMatrix(1, 1),
      newCameraMatrix(0, 2),
      newCameraMatrix(1, 2),
      width, height,
      typename ideal_pinhole_geometry_t::projection_t::distortion_t()
  );

  _idealPinholeGeometry = boost::shared_ptr<ideal_pinhole_geometry_t>(
      new ideal_pinhole_geometry_t(idealPinholeProjection,
                                   _distortedGeometry->shutter(),
                                   _distortedGeometry->mask()
                                   )
  );
}

template<typename MASK_T>
boost::shared_ptr<FrameBase> OmniUndistorter<MASK_T>::buildFrame(
    cv::Mat & inImage) {

  boost::shared_ptr<frame_t> frame(new frame_t());
  constructUndistortedFrame(inImage, *frame);

  return frame;
}

}  // namespace aslam
