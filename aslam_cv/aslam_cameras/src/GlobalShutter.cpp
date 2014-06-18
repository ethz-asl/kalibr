#include <aslam/cameras/GlobalShutter.hpp>
#include <sm/PropertyTree.hpp>
namespace aslam {
namespace cameras {
GlobalShutter::GlobalShutter() {

}

GlobalShutter::GlobalShutter(const sm::PropertyTree &) {

}

GlobalShutter::~GlobalShutter() {

}

// aslam::backend compatibility
void GlobalShutter::update(const double *) {

}

int GlobalShutter::minimalDimensions() const {
  return 0;
}

void GlobalShutter::getParameters(Eigen::MatrixXd & P) const {
  P.resize(0, 0);
}

void GlobalShutter::setParameters(const Eigen::MatrixXd & /* P */) {

}

Eigen::Vector2i GlobalShutter::parameterSize() const {
  return Eigen::Vector2i(0, 0);
}

}  // namespace cameras
}  // namespace aslam
