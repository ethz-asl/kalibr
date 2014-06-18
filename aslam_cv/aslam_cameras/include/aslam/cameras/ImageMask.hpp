#ifndef ASLAM_CAMERAS_IMAGE_MASK_HPP
#define ASLAM_CAMERAS_IMAGE_MASK_HPP

#include <sm/opencv/serialization.hpp>
#include <Eigen/Core>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

class ImageMask {
 public:
  ImageMask();
  ImageMask(const sm::PropertyTree & config);
  ImageMask(const cv::Mat& mask, double scale = 1.0);
  ~ImageMask();

  void setMask(const cv::Mat& mask);
  const cv::Mat & getMask() const;

  void setScale(double scale);
  double getScale() const;

  // These guys mainly support the python interface
  void setMaskFromMatrix(const Eigen::MatrixXi & mask);
  Eigen::MatrixXi getMaskAsMatrix() const;

  template<typename DERIVED>
  bool isValid(const Eigen::MatrixBase<DERIVED> & k) const {
    int k1 = k(1, 0) * _scale;
    int k0 = k(0, 0) * _scale;
    // \todo fix this when it is initialized properly
    return !_mask.data || (_mask.at<unsigned char>(k1, k0) > 0);
    //return true;
  }

  // is the mask set? (i.e. mask data != NULL)
  bool isSet () const;

  bool isBinaryEqual(const ImageMask & rhs) const;

  enum {
    CLASS_SERIALIZATION_VERSION = 1
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int) CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");

    if (CLASS_SERIALIZATION_VERSION < 1) {
      ar >> BOOST_SERIALIZATION_NVP(_mask);
      _scale = 1.0;
    } else {
      ar >> BOOST_SERIALIZATION_NVP(_mask);
      ar >> BOOST_SERIALIZATION_NVP(_scale);
    }
  }

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(_mask);
    ar << BOOST_SERIALIZATION_NVP(_scale);
  }

 private:
  cv::Mat _mask;
  double _scale;
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION (aslam::cameras::ImageMask);

#endif /* ASLAM_CAMERAS_IMAGE_MASK_HPP */
