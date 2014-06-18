#ifndef ASLAM_NULL_UNDISTORTER_HPP
#define ASLAM_NULL_UNDISTORTER_HPP

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

/// \brief A class that looks like an undistorter but does nothing.
template<typename CAMERA_T>
class NullUndistorter : public UndistorterBase {
 public:
  typedef CAMERA_T distorted_geometry_t;
  typedef CAMERA_T ideal_geometry_t;
  typedef Frame<ideal_geometry_t> frame_t;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NullUndistorter();

  NullUndistorter(const sm::PropertyTree& undistorterConfig,
                  const sm::PropertyTree& cameraConfig);

  /// \param[in] cameraConfig         A single camera part of a Property Tree is expected.
  /// \param[in] scale                Allows to scale the resulting image.
  /// \param[in] makeCopy             If the image is not scaled and one still wants a copy.
  NullUndistorter(const sm::PropertyTree& cameraConfig, int interpolation,
                  double scale, bool makeCopy);
  /// \param[in] scale                Allows to scale the resulting image.
  /// \param[in] makeCopy             If the image is not scaled and one still wants a copy.
  NullUndistorter(boost::shared_ptr<distorted_geometry_t> distortedGeometry,
                  int interpolation, double scale, bool makeCopy);
  /// \param[in] distortedGeometry    the distorted geometry type
  /// \param[in] config               a configuration object for the other parameters
  /// \param[in] makeCopy             If the image is not scaled and one still wants a copy.
  NullUndistorter(boost::shared_ptr<distorted_geometry_t> distortedGeometry,
                  const sm::PropertyTree & config);
  ~NullUndistorter();

  /// \brief Initialize the Undistorter.
  /// \param[in] scale                Allows to scale the resulting image.
  void init(boost::shared_ptr<distorted_geometry_t> distortedGeometry,
            int interpolation, double scale, bool makeCopy);

  /// \brief this undistorts the image and sets both the image and the geometry in the out frame.
  void constructUndistortedFrame(const cv::Mat & inImage,
                                 frame_t & outFrame) const;

  /// \brief Undistort a single image.
  void undistortImage(const cv::Mat & inImage, cv::Mat & outImage) const;

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
      boost::shared_ptr<distorted_geometry_t> distortedGeometry);

  boost::shared_ptr<ideal_geometry_t> _idealGeometry;
  int _interpolation;
  double _scale;
  bool _makeCopy;
};

}  // namespace aslam

#include "implementation/NullUndistorter.hpp"

#endif /* ASLAM_NULL_UNDISTORTER_HPP */
