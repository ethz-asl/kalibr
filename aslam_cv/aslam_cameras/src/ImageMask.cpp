#include <aslam/cameras/ImageMask.hpp>
#include <sm/assert_macros.hpp>
#include <sm/PropertyTree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sm/serialization_macros.hpp>

namespace aslam {
namespace cameras {

ImageMask::ImageMask()
    : _mask(cv::Mat(0, 0, CV_8UC1)),
      _scale(1.0)
{}
;

ImageMask::ImageMask(const sm::PropertyTree & config)
    : _scale(1.0) {
  std::string maskFile = config.getString("mask-file");
  // Note: if this fails, _mask.data == NULL.
  // http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html#cv-imread
  // \todo Better error handling.
  _mask = cv::imread(maskFile, CV_LOAD_IMAGE_GRAYSCALE);
  //TODO: BB: How to implement scale here?
}

ImageMask::ImageMask(const cv::Mat& mask, double scale) {
  SM_ASSERT_EQ(std::runtime_error, mask.type(), CV_8UC1,
               "The mask must be 8-bit one channel");
  _mask = mask.clone();
  _scale = scale;
}

ImageMask::~ImageMask() {
}
;

void ImageMask::setMask(const cv::Mat& mask) {
  SM_ASSERT_EQ(std::runtime_error, mask.type(), CV_8UC1,
               "The mask must be 8-bit one channel");
  _mask = mask.clone();
}

bool ImageMask::isSet() const
{
	return _mask.data != NULL;
}

const cv::Mat & ImageMask::getMask() const {
  return _mask;
}

void ImageMask::setScale(double scale) {
  _scale = scale;
}

double ImageMask::getScale() const {
  return _scale;
}

bool ImageMask::isBinaryEqual(const ImageMask & rhs) const {
  return sm::opencv::isBinaryEqual(_mask, rhs._mask);
}

void ImageMask::setMaskFromMatrix(const Eigen::MatrixXi & mask) {
  _mask.create(mask.rows(), mask.cols(), CV_8UC1);
  for (int r = 0; r < mask.rows(); ++r) {
    uchar * p = _mask.ptr(r);
    for (int c = 0; c < mask.cols(); ++c, ++p) {
      *p = mask(r, c);
    }
  }
}

Eigen::MatrixXi ImageMask::getMaskAsMatrix() const {
  Eigen::MatrixXi rval(_mask.rows, _mask.cols);

  for (int r = 0; r < rval.rows(); ++r) {
    const uchar * p = _mask.ptr(r);
    for (int c = 0; c < rval.cols(); ++c, ++p) {
      rval(r, c) = *p;
    }
  }
  return rval;

}

}  // namespace cameras
}  // namespace aslam
