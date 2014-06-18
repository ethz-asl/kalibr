#include <sm/opencv/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <aslam/Image.hpp>

namespace aslam {

Image::Image(int octaves) {
  _octaves.resize(octaves);
}

Image::~Image() {
}

void Image::setNumOctaves(size_t octaves) {
  SM_ASSERT_GT_DBG(IndexOutOfBoundsException, octaves, 0,
                   "Image octave number must be greater than zero");
  _octaves.resize(octaves);
}

bool Image::isBinaryEqual(const Image& lhs) const {
  bool good = SM_CHECKSAME(_octaves.size(), lhs._octaves.size());

  for (size_t i = 0; good && i < _octaves.size(); ++i) {
    good = good && sm::opencv::isBinaryEqual(_octaves[i], lhs._octaves[i]);

  }
  return good;
}

}  //namespace aslam

