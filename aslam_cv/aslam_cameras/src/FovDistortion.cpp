#include <aslam/cameras/FovDistortion.hpp>
#include <sm/serialization_macros.hpp>
namespace aslam {
namespace cameras {

FovDistortion::FovDistortion()
    : _w(1.0) {
  SM_ASSERT_TRUE(std::runtime_error, areParametersValid(_w), "Invalid distortion parameter");
}

FovDistortion::FovDistortion(double w)
    : _w(w){
  SM_ASSERT_TRUE(std::runtime_error, areParametersValid(w), "Invalid distortion parameter");
}

FovDistortion::FovDistortion(const sm::PropertyTree & config) {
  _w = config.getDouble("w");
  SM_ASSERT_TRUE(std::runtime_error, areParametersValid(_w), "Invalid distortion parameter");
}

FovDistortion::~FovDistortion() {}

// aslam::backend compatibility
void FovDistortion::update(const double * v) {
  _w += v[0];
  SM_ASSERT_TRUE(std::runtime_error, areParametersValid(_w), "Invalid distortion parameter");
}

int FovDistortion::minimalDimensions() const {
  return IntrinsicsDimension;
}

void FovDistortion::getParameters(Eigen::MatrixXd & S) const {
  S.resize(1, 1);
  S(0, 0) = _w;
}

void FovDistortion::setParameters(const Eigen::MatrixXd & S) {
  _w = S(0, 0);
  SM_ASSERT_TRUE(std::runtime_error, areParametersValid(_w), "Invalid distortion parameter");
}

Eigen::Vector2i FovDistortion::parameterSize() const {
  return Eigen::Vector2i(1, 1);
}

bool FovDistortion::isBinaryEqual(
    const FovDistortion & rhs) const {
  return SM_CHECKMEMBERSSAME(rhs, _w);
}

FovDistortion FovDistortion::getTestDistortion() {
  return FovDistortion(1.0);
}

void FovDistortion::clear() {
  _w = 1.0;
}

}  // namespace cameras
}  // namespace aslam
