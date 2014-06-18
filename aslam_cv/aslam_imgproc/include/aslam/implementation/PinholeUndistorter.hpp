#include <boost/make_shared.hpp>
#include "aslamcv_helper.hpp"

namespace aslam {

template<typename DISTORTION_T, typename MASK_T>
PinholeUndistorter<DISTORTION_T, MASK_T>::PinholeUndistorter() {
  _idealGeometry.reset();
}

template<typename DISTORTION_T, typename MASK_T>
PinholeUndistorter<DISTORTION_T, MASK_T>::PinholeUndistorter(
    const sm::PropertyTree& undistorterConfig,
    const sm::PropertyTree& cameraConfig)
{
  boost::shared_ptr<distorted_geometry_t> distortedGeometry(  new distorted_geometry_t(cameraConfig));

  int interpolation = undistorterConfig.getInt("interpolationType", cv::INTER_NEAREST);
  double alpha = undistorterConfig.getDouble("alpha");
  double scale = undistorterConfig.getDouble("scale");
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename DISTORTION_T, typename MASK_T>
PinholeUndistorter<DISTORTION_T, MASK_T>::PinholeUndistorter(
    const sm::PropertyTree& cameraConfig, int interpolation, double alpha,
    double scale) {
  boost::shared_ptr<distorted_geometry_t> distortedGeometry(
      new distorted_geometry_t(cameraConfig));
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename DISTORTION_T, typename MASK_T>
PinholeUndistorter<DISTORTION_T, MASK_T>::PinholeUndistorter(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    int interpolation, double alpha, double scale) {
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename DISTORTION_T, typename MASK_T>
PinholeUndistorter<DISTORTION_T, MASK_T>::PinholeUndistorter(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    const sm::PropertyTree & config) {
  int interpolation = config.getInt("interpolationType", cv::INTER_NEAREST);
  double alpha = config.getDouble("alpha");
  double scale = config.getDouble("scale");
  init(distortedGeometry, interpolation, alpha, scale);
}

template<typename DISTORTION_T, typename MASK_T>
PinholeUndistorter<DISTORTION_T, MASK_T>::~PinholeUndistorter() {
}

template<typename DISTORTION_T, typename MASK_T>
void PinholeUndistorter<DISTORTION_T, MASK_T>::init(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry,
    int interpolation, double alpha, double scale) {
  _interpolation = interpolation;
  setIdealGeometry(distortedGeometry, alpha, scale);
}

template<typename DISTORTION_T, typename MASK_T>
void PinholeUndistorter<DISTORTION_T, MASK_T>::constructUndistortedFrame(
    const cv::Mat & image, frame_t & outFrame) const {
  SM_ASSERT_TRUE(std::runtime_error, _idealGeometry,
                 "Camera has not yet been set.")
  cv::Mat undistImage;
  undistortImage(image, undistImage);
  outFrame.setImage(undistImage);
  outFrame.setGeometry(_idealGeometry);
}

template<typename DISTORTION_T, typename MASK_T>
void PinholeUndistorter<DISTORTION_T, MASK_T>::undistortImage(const cv::Mat & inImage,
                                                cv::Mat & outImage) const {
  // see: http://opencv.willowgarage.com/documentation/cpp/imgproc_geometric_image_transformations.html#remap
  cv::remap(inImage, outImage, mapX, mapY, _interpolation);
}

template<typename DISTORTION_T, typename MASK_T>
boost::shared_ptr<typename PinholeUndistorter<DISTORTION_T, MASK_T>::ideal_geometry_t> PinholeUndistorter<
  DISTORTION_T, MASK_T>::idealGeometry() const {
  return _idealGeometry;
}

template<typename DISTORTION_T, typename MASK_T>
boost::shared_ptr<typename PinholeUndistorter<DISTORTION_T, MASK_T>::distorted_geometry_t> PinholeUndistorter<
DISTORTION_T, MASK_T>::distortedGeometry() const {
  return _distortedGeometry;
}

template<typename DISTORTION_T, typename MASK_T>
void PinholeUndistorter<DISTORTION_T, MASK_T>::setIdealGeometry(
    boost::shared_ptr<distorted_geometry_t> distortedGeometry, double alpha,
    double scale) {

  //set distorted geometry
  _distortedGeometry = distortedGeometry;

  int width = distortedGeometry->projection().ru();
  int height = distortedGeometry->projection().rv();
  int newWidth = (int) width * scale;
  int newHeight = (int) height * scale;

  // compute the optimal new camera matrix based on the free scaling parameter
  // see: http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#getOptimalNewCameraMatrix
  Eigen::Matrix3d idealCameraMatrix = Eigen::Matrix3d::Zero();
  idealCameraMatrix = aslamcv_helper::getOptimalNewCameraMatrix<distorted_geometry_t>(
                                                        _distortedGeometry, cv::Size(width, height),
                                                        alpha, cv::Size(newWidth, newHeight));

  // create idealProjection
  typename ideal_geometry_t::projection_t idealProjection(
      idealCameraMatrix(0, 0),
      idealCameraMatrix(1, 1),
      idealCameraMatrix(0, 2),
      idealCameraMatrix(1, 2),
      newWidth,
      newHeight,
      typename ideal_geometry_t::projection_t::distortion_t()
  );

  // set new idealGeometry
  _idealGeometry = boost::make_shared<ideal_geometry_t>(idealProjection,
                                                        _distortedGeometry->shutter(),
                                                        _distortedGeometry->mask() );

  // compute the undistortion and rectification transformation map
  // see: http://opencv.willowgarage.com/documentation/cpp/calib3d_camera_calibration_and_3d_reconstruction.html#initUndistortRectifyMap
  aslamcv_helper::initUndistortRectifyMap<distorted_geometry_t>(_distortedGeometry,
                                                                Eigen::Matrix3d::Identity(),
                                                                idealCameraMatrix,
                                                                cv::Size(newWidth, newHeight),
                                                                CV_16SC2, mapX, mapY);

}

template<typename DISTORTION_T, typename MASK_T>
boost::shared_ptr<FrameBase> PinholeUndistorter<DISTORTION_T, MASK_T>::buildFrame(
    cv::Mat & inImage) {

  boost::shared_ptr < frame_t > frame(new frame_t());
  constructUndistortedFrame(inImage, *frame);

  return frame;
}

}  // namespace aslam
