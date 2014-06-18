#ifndef ASLAM_UNDISTORTER_BASE_HPP
#define ASLAM_UNDISTORTER_BASE_HPP

#include <aslam/cameras/CameraGeometryBase.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sm/PropertyTree.hpp>

namespace aslam {

namespace interpolation {

enum InterpolationType {
  NearestNeighbor = cv::INTER_NEAREST,  // nearest-neighbor interpolation
  Linear = cv::INTER_LINEAR,         // bilinear interpolation (used by default)
  Area = cv::INTER_AREA,  // resampling using pixel area relation. It may be the preferred method for image decimation, as it gives moire-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method
  Cubic = cv::INTER_CUBIC,          // Interpolation over 4x4 pixel neighborhood
  Lanczos4 = cv::INTER_LANCZOS4  // Lanczos interpolation over 8x8 pixel neighborhood
};

}  // namespace interpolation

typedef interpolation::InterpolationType InterpolationType;

class UndistorterBase {
 public:
  UndistorterBase();
  virtual ~UndistorterBase();

  static boost::shared_ptr<UndistorterBase> createUndistorter(
      const sm::PropertyTree & undistorterConfig,
      const sm::PropertyTree & cameraConfig);

  /// \brief Given the underlying camera geometry, undistort the image
  virtual void undistortImage(const cv::Mat & inImage,
                              cv::Mat & outImage) const = 0;

  /// \brief Undistort the image and pack it into a frame class. 
  virtual boost::shared_ptr<FrameBase> buildFrame(cv::Mat & inImage) = 0;

  /// \brief Get the ideal geometry (after undistortion). This is the geometry
  ///        that will be packed into the frame when calling buildFrame()
  virtual boost::shared_ptr<cameras::CameraGeometryBase> idealGeometryBase() const = 0;

  /// \brief Get the distorted geometr (before undistortion)
  virtual boost::shared_ptr<cameras::CameraGeometryBase> distortedGeometryBase() const = 0;
};

}  // namespace aslam

#endif /* ASLAM_UNDISTORTER_BASE_HPP */
