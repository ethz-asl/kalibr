#ifndef ASLAM_OMNI_UNDISTORTER_HPP
#define ASLAM_OMNI_UNDISTORTER_HPP

#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras.hpp>
#include <aslam/Frame.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/assert_macros.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/UndistorterBase.hpp>

namespace cv {
class Mat;
}  // namespace cv

namespace aslam {

/// \brief A class for undistorting Images with the Omni Camera Model.
template<typename MASK_T>
class OmniUndistorter : public UndistorterBase {
 public:
  typedef cameras::CameraGeometry<
      cameras::OmniProjection<cameras::RadialTangentialDistortion>,
      cameras::GlobalShutter, MASK_T> distorted_geometry_t;
  typedef cameras::CameraGeometry<
      cameras::OmniProjection<cameras::NoDistortion>, cameras::GlobalShutter,
      MASK_T> ideal_geometry_t;
  typedef cameras::CameraGeometry<
      cameras::PinholeProjection<cameras::NoDistortion>, cameras::GlobalShutter,
      MASK_T> ideal_pinhole_geometry_t;
  typedef Frame<ideal_geometry_t> frame_t;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OmniUndistorter();
  OmniUndistorter(const sm::PropertyTree& undistorterConfig,
                  const sm::PropertyTree& cameraConfig);
  /// \param[in] cameraConfig         A single camera part of a Property Tree is expected.
  /// \param[in] alpha                The free scaling parameter between 0 (when all the pixels in the undistorted image will be valid) and 1 (when all the source image pixels will be retained in the undistorted image).
  /// \param[in] scale                Allows to scale the resulting image.
  OmniUndistorter(const sm::PropertyTree& cameraConfig, int interpolation,
                  double alpha, double scale);
  /// \param[in] alpha                The free scaling parameter between 0 (when all the pixels in the undistorted image will be valid) and 1 (when all the source image pixels will be retained in the undistorted image).
  /// \param[in] scale                Allows to scale the resulting image.
  OmniUndistorter(boost::shared_ptr<distorted_geometry_t> distortedGeometry,
                  int interpolation, double alpha, double scale);
  /// \param[in] distortedGeometry    the distorted geometry type
  /// \param[in] config               a configuration object for the other parameters
  OmniUndistorter(boost::shared_ptr<distorted_geometry_t> distortedGeometry,
                  const sm::PropertyTree & config);
  ~OmniUndistorter();

  /// \brief Initialize the Undistorter.
  /// \param[in] alpha                The free scaling parameter between 0 (when all the pixels in the undistorted image will be valid) and 1 (when all the source image pixels will be retained in the undistorted image).
  /// \param[in] scale                Allows to scale the resulting image.
  void init(boost::shared_ptr<distorted_geometry_t> distortedGeometry,
            int interpolation, double alpha, double scale);

  /// \brief this undistorts the image and sets both the image and the geometry in the out frame.
  void constructUndistortedFrame(const cv::Mat & inImage,
                                 frame_t & outFrame) const;

  /// \brief Undistort a single image.
  void undistortImage(const cv::Mat & inImage, cv::Mat & outImage) const;
  void undistortImageToPinhole(const cv::Mat & inImage, cv::Mat & outImage) const;

  boost::shared_ptr<ideal_pinhole_geometry_t> idealPinholeGeometry() const;
  boost::shared_ptr<cameras::CameraGeometryBase> idealPinholeGeometryBase() const {
    return _idealPinholeGeometry();
  }

  boost::shared_ptr<ideal_geometry_t> idealGeometry() const;
  boost::shared_ptr<cameras::CameraGeometryBase> idealGeometryBase() const {
    return idealGeometry();
  }

  boost::shared_ptr<distorted_geometry_t> distortedGeometry() const;
  boost::shared_ptr<cameras::CameraGeometryBase> distortedGeometryBase() const {
    return distortedGeometry();
  }

  virtual boost::shared_ptr<FrameBase> buildFrame(cv::Mat & inImage);

 private:
  /// \brief Set ideal geometry and create undistortion maps.
  void setIdealGeometry(
      boost::shared_ptr<distorted_geometry_t> distortedGeometry, double alpha,
      double scale);

  /// \brief Set ideal pinhole geometry and create omni to pinhole maps.
  void setIdealPinholeGeometry(boost::shared_ptr<ideal_geometry_t> idealGeometry);

  boost::shared_ptr<ideal_geometry_t> _idealGeometry;
  boost::shared_ptr<ideal_pinhole_geometry_t> _idealPinholeGeometry;
  boost::shared_ptr<distorted_geometry_t> _distortedGeometry;

  //mapping distorted omni --> undistorted omni
  cv::Mat mapX, mapY;

  //mapping undistorted omni --> undistorted pinhole
  cv::Mat mapX_pinhole, mapY_pinhole;

  int _interpolation;
};

}  // namespace aslam

#include "implementation/OmniUndistorter.hpp"

#endif /* ASLAM_OMNI_UNDISTORTER_HPP */
