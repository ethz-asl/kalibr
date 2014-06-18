#include <aslam/cameras/EquidistantDistortion.hpp>
#include <sm/serialization_macros.hpp>
namespace aslam {
namespace cameras {

EquidistantDistortion::EquidistantDistortion()
    : _k1(0),
      _k2(0),
      _k3(0),
      _k4(0) {

}

EquidistantDistortion::EquidistantDistortion(double k1, double k2, double k3,
                                             double k4)
    : _k1(k1),
      _k2(k2),
      _k3(k3),
      _k4(k4) {

}

EquidistantDistortion::EquidistantDistortion(const sm::PropertyTree & config) {
  _k1 = config.getDouble("k1");
  _k2 = config.getDouble("k2");
  _k3 = config.getDouble("k3");
  _k4 = config.getDouble("k4");
}

EquidistantDistortion::~EquidistantDistortion() {

}

// aslam::backend compatibility
void EquidistantDistortion::update(const double * v) {
  _k1 += v[0];
  _k2 += v[1];
  _k3 += v[2];
  _k4 += v[3];
}

int EquidistantDistortion::minimalDimensions() const {
  return IntrinsicsDimension;
}

void EquidistantDistortion::getParameters(Eigen::MatrixXd & S) const {
  S.resize(4, 1);
  S(0, 0) = _k1;
  S(1, 0) = _k2;
  S(2, 0) = _k3;
  S(3, 0) = _k4;
}

void EquidistantDistortion::setParameters(const Eigen::MatrixXd & S) {
  _k1 = S(0, 0);
  _k2 = S(1, 0);
  _k3 = S(2, 0);
  _k4 = S(3, 0);
}

Eigen::Vector2i EquidistantDistortion::parameterSize() const {
  return Eigen::Vector2i(4, 1);
}

bool EquidistantDistortion::isBinaryEqual(
    const EquidistantDistortion & rhs) const {
  return SM_CHECKMEMBERSSAME(rhs, _k1) && SM_CHECKMEMBERSSAME(rhs, _k2)
      && SM_CHECKMEMBERSSAME(rhs, _k3) && SM_CHECKMEMBERSSAME(rhs, _k4);
}

EquidistantDistortion EquidistantDistortion::getTestDistortion() {
  return EquidistantDistortion(-0.2, 0.13, 0.0005, 0.0005);
}

void EquidistantDistortion::clear() {
  _k1 = 0.0;
  _k2 = 0.0;
  _k3 = 0.0;
  _k4 = 0.0;
}

}  // namespace cameras
}  // namespace aslam
