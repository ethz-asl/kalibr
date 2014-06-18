#include <aslam/backend/MEstimatorPolicies.hpp>

#include <cmath>

#include <sstream>

#include <boost/math/distributions/chi_squared.hpp>

namespace aslam {
namespace backend {

MEstimator::~MEstimator() {
}

NoMEstimator::~NoMEstimator() {
}
double NoMEstimator::getWeight(double /* squaredError */) const {
  return 1.0;
}
std::string NoMEstimator::name() const {
  return "none";
}

GemanMcClureMEstimator::GemanMcClureMEstimator(double sigma2) :
    _sigma2(sigma2) {
}
GemanMcClureMEstimator::~GemanMcClureMEstimator()
{
}
double GemanMcClureMEstimator::getWeight(double error) const {
  double se = _sigma2 + error;
  return (_sigma2) / (se * se);
}
std::string GemanMcClureMEstimator::name() const {
  std::stringstream ss;
  ss << "Geman McClure (" << _sigma2 << ")";
  return ss.str();
}

CauchyMEstimator::CauchyMEstimator(double sigma2) :
    _sigma2(sigma2) {
}
CauchyMEstimator::~CauchyMEstimator()
{
}
double CauchyMEstimator::getWeight(double error) const {
  double se = error / _sigma2;
      return 1.0 / (1.0 + se);
}
std::string CauchyMEstimator::name() const {
  std::stringstream ss;
  ss << "Cauchy (" << _sigma2 << ")";
  return ss.str();
}


  
HuberMEstimator::~HuberMEstimator()
{
}
HuberMEstimator::HuberMEstimator(double k) :
    _k(k), _k2(k * k) {
}
double HuberMEstimator::getWeight(double error) const {
  return error < _k2 ? 1.0 : _k / sqrt(error);
}
std::string HuberMEstimator::name() const {
  std::stringstream ss;
  ss << "Huber(" << _k << ")";
  return ss.str();
}

BlakeZissermanMEstimator::~BlakeZissermanMEstimator() {
}
BlakeZissermanMEstimator::BlakeZissermanMEstimator(size_t df, double pCut,
                                                   double wCut) :
    _df(df),
    _pCut(pCut),
    _wCut(wCut),
    _epsilon(computeEpsilon(df, pCut, wCut)) {
}
BlakeZissermanMEstimator::BlakeZissermanMEstimator(const
                                                   BlakeZissermanMEstimator& other) :
    MEstimator(other),
    _df(other._df),
    _pCut(other._pCut),
    _wCut(other._wCut),
    _epsilon(other._epsilon) {
}
BlakeZissermanMEstimator& BlakeZissermanMEstimator::operator =
(const BlakeZissermanMEstimator& other) {
  if (this != &other) {
    MEstimator::operator=(other);
    _df = other._df;
    _pCut = other._pCut;
    _wCut = other._wCut;
    _epsilon = other._epsilon;
  }
  return *this;
}
double BlakeZissermanMEstimator::getWeight(double mahalanobis2) const {
  return exp(-mahalanobis2) / (exp(-mahalanobis2) + _epsilon);
}
std::string BlakeZissermanMEstimator::name() const {
  std::stringstream ss;
  ss << "Blake-Zisserman(" << _epsilon << ")";
  return ss.str();
}
double BlakeZissermanMEstimator::chi2InvCDF(double p, size_t df) const {
  return boost::math::quantile(boost::math::chi_squared_distribution<>(df),
                               p);
}
double BlakeZissermanMEstimator::computeEpsilon(size_t df, double pCut,
                                                double wCut) const {
  return (1 - wCut) / wCut * exp(-chi2InvCDF(pCut, df));
}

} // namespace backend
} // namespace aslam
