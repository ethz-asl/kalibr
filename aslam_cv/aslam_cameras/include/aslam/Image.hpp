#ifndef ASLAM_IMAGE_HPP_
#define ASLAM_IMAGE_HPP_

#include <vector>

#include <sm/assert_macros.hpp>
#include <opencv2/core/core.hpp>
#include <boost/serialization/split_member.hpp>
#include <sm/boost/serialization.hpp>

namespace aslam {

class Image {
 private:
  std::vector<cv::Mat> _octaves;
 public:

  SM_DEFINE_EXCEPTION(RTException, std::runtime_error);SM_DEFINE_EXCEPTION(IndexOutOfBoundsException, RTException);

  Image(int octaves = 1);
  virtual ~Image();

  size_t numOctaves() const;
  void setNumOctaves(size_t octaves);
  inline const cv::Mat& getOctave(int octave) const;
  inline cv::Mat& getOctaveMutable(int octave);
  bool isBinaryEqual(const Image& lhs) const;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version);

  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

};
} /* namespace aslam */
SM_BOOST_CLASS_VERSION (aslam::Image);

#include "implementation/Image.hpp"
#endif /* ASLAM_IMAGE_HPP_ */
