#include "apriltags/Gaussian.h"
#include <iostream>

namespace AprilTags {

bool Gaussian::warned = false;

std::vector<float> Gaussian::makeGaussianFilter(float sigma, int n) {
  std::vector<float> f(n,0.0f);

  if (sigma == 0) {
    for (int i = 0; i < n; i++)
      f[i] = 0;
    f[n/2] = 1;
    return f;
  }

  double const inv_variance = 1./(2*sigma*sigma);
  float sum = 0;
  for (int i = 0; i < n; i++) {
    int j = i - n/2;
    f[i] = (float)std::exp(-j*j * inv_variance);
    sum += f[i];
  }

  // normalize the gaussian
  for (int i = 0; i < n; i++)
    f[i] /= sum;

  return f;
}

void Gaussian::convolveSymmetricCentered(const std::vector<float>& a, unsigned int aoff, unsigned int alen,
					const std::vector<float>& f, std::vector<float>& r, unsigned int roff) {
  if ((f.size()&1)== 0 && !warned) {
    std::cout<<"convolveSymmetricCentered Warning: filter is not odd length\n";
    warned = true;
  }

  for (size_t i = f.size()/2; i < f.size(); i++) {
    double acc = 0;
    for (size_t j = 0; j < f.size(); j++) {
      if ((aoff + i) < j || (aoff + i) >= (alen + j))
	acc += a[aoff] * f[j];
      else
	acc += a[aoff + i - j] * f[j];
    }
    r[roff + i - f.size()/2] = (float)acc;
  }

  for (size_t i = f.size(); i < alen; i++) {
    double acc = 0;
    for (unsigned int j = 0; j < f.size(); j++) {
      acc += a[aoff + i - j] * f[j];
    }
    r[roff + i - f.size()/2] = (float)acc;
  }

  for (size_t i = alen; i < alen + f.size()/2; i++) {
    double acc = 0;
    for (size_t j = 0; j < f.size(); j++) {
      if ((aoff + i) >= (alen + j) || (aoff + i) < j)
	acc += a[aoff + alen - 1] * f[j];
      else
	acc += a[aoff + i - j] * f[j];
    }
    r[roff + i - f.size()/2] = (float)acc;
  }
}

} // namespace
