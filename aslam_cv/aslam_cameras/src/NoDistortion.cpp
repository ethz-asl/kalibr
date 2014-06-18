#include <aslam/cameras/NoDistortion.hpp>
#include <sm/PropertyTree.hpp>
namespace aslam {
namespace cameras {

NoDistortion::NoDistortion() {
}
NoDistortion::NoDistortion(const sm::PropertyTree &) {
}
NoDistortion::~NoDistortion() {
}

// aslam::backend compatibility
void NoDistortion::update(const double *) {

}
int NoDistortion::minimalDimensions() const {
  return 0;
}
void NoDistortion::getParameters(Eigen::MatrixXd & P) const {
  P.resize(0, 0);
}
void NoDistortion::setParameters(const Eigen::MatrixXd & /* P */) {
}

Eigen::Vector2i NoDistortion::parameterSize() const {
  return Eigen::Vector2i(0, 0);
}

}  // namespace cameras
}  // namespace aslam
