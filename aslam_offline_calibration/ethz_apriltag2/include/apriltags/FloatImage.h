#ifndef FLOATIMAGE_H
#define FLOATIMAGE_H

#include <algorithm>
#include <vector>

namespace DualCoding {
	typedef unsigned char uchar;
	template<typename T> class Sketch;
}

namespace AprilTags {

//! Represent an image as a vector of floats in [0,1]
class FloatImage {
private:
  int width;
  int height;
  std::vector<float> pixels;

public:

  //! Default constructor
  FloatImage();

  //! Construct an empty image
  FloatImage(int widthArg, int heightArg);

  //! Constructor that copies pixels from an array
  FloatImage(int widthArg, int heightArg, const std::vector<float>& pArg);

  FloatImage& operator=(const FloatImage& other);

  float get(int x, int y) const { return pixels[y*width + x]; }
  void set(int x, int y, float v) { pixels[y*width + x] = v; }
  
  int getWidth() const { return width; }
  int getHeight() const { return height; }
  int getNumFloatImagePixels() const { return width*height; }
  const std::vector<float>& getFloatImagePixels() const { return pixels; }

  //! TODO: Fix decimateAvg function. DO NOT USE!
  void decimateAvg();

  //! Rescale all values so that they are between [0,1]
  void normalize();

  void filterFactoredCentered(const std::vector<float>& fhoriz, const std::vector<float>& fvert);

  template<typename T>
  void copyToSketch(DualCoding::Sketch<T>& sketch) {
    for (int i = 0; i < getNumFloatImagePixels(); i++)
      sketch[i] = (T)(255.0f * getFloatImagePixels()[i]);
  }

  void printMinMax() const;
};

} // namespace

#endif
