#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <cmath>
#include <vector>

namespace AprilTags {

class Gaussian {

public:
  static bool warned;

  //! Returns a Gaussian filter of size n.
  /*! @param sigma standard deviation of the Gaussian
   *  @param n length of the Gaussian (must be odd)
   */
  static std::vector<float> makeGaussianFilter(float sigma, int n);

  //! Convolve the input 'a' (which begins at offset aoff and is alen elements in length) with the filter 'f'.
  /*! The result is deposited in 'r' at offset 'roff'. f.size() should be odd.
   *  The output is shifted by -f.size()/2, so that there is no net time delay.
   *  @param a input vector of pixels
   *  @param aoff
   *  @param alen
   *  @param f
   *  @param r the resultant array of pixels
   *  @param roff 
   */
  static void convolveSymmetricCentered(const std::vector<float>& a, unsigned int aoff, unsigned int alen,
					const std::vector<float>& f, std::vector<float>& r, unsigned int roff);
  
};

} // namespace

#endif
