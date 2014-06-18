#include <sm/opencv/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace aslam {

inline const cv::Mat& Image::getOctave(int octave) const {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, (static_cast<size_t>(octave)),
                   (_octaves.size()), "Image octave index out of bounds");
  return _octaves[octave];
}

inline cv::Mat& Image::getOctaveMutable(int octave) {
  SM_ASSERT_LT_DBG(IndexOutOfBoundsException, (static_cast<size_t>(octave)),
                   _octaves.size(), "Image octave index out of bounds");
  return _octaves[octave];
}

template<class Archive>
void Image::load(Archive & ar, const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");
  ar >> BOOST_SERIALIZATION_NVP(_octaves);
}

template<class Archive>
void Image::save(Archive & ar, const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_octaves);
}

} /* namespace aslam */
