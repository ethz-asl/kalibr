#ifndef _SAMPLEDVANDERROR_H_
#define _SAMPLEDVANDERROR_H_

#include <sm/random.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <sm/eigen/gtest.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

class Point2d : public aslam::backend::DesignVariable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d _v;
  Eigen::Vector2d _p_v;

  Point2d(const Eigen::Vector2d& v) : _v(v), _p_v(v) {}
  virtual ~Point2d() {}

protected:
  /// \brief Revert the last state update.
  virtual void revertUpdateImplementation() {
    _v = _p_v;
  }

  /// \brief Update the design variable.
  virtual void updateImplementation(const double* dp, int /* size */) {
    _p_v = _v;
    _v[0] += dp[0];
    _v[1] += dp[1];
  }

  /// \brief what is the number of dimensions of the perturbation variable.
  virtual int minimalDimensionsImplementation() const {
    return 2;
  }

  /// Returns the content of the design variable
  virtual void getParametersImplementation(Eigen::MatrixXd& value) const {
    value = _v;
  }

  /// Sets the content of the design variable
  virtual void setParametersImplementation(const Eigen::MatrixXd& value) {
    _p_v = _v;
    _v = value;
  }

};

class LinearErr : public aslam::backend::ErrorTermFs<2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef aslam::backend::ErrorTermFs<2> parent_t;

  Eigen::Vector2d _p;
  Eigen::Matrix2d _J;
  Point2d* _p2d;

  LinearErr(Point2d* p2d) : _p2d(p2d) {
    _p2d->setActive(true);
    _J.setRandom();
    _p = _J * _p2d->_v;
    parent_t::setDesignVariables(_p2d);
    setInvR(sm::eigen::randomCovariance<2>());
    _p[0] += sm::random::randn() * 0.1;
    _p[1] += sm::random::randn() * 0.1;
  }
  virtual ~LinearErr() {}

  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    setError(_p - _J * _p2d->_v);
    return evaluateChiSquaredError();
  }

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJ) const {
    outJ.add(_p2d, -_J);
  }

};


class LinearErr2 : public aslam::backend::ErrorTermFs<2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef aslam::backend::ErrorTermFs<2> parent_t;

  Eigen::Vector2d _p;
  Eigen::Matrix2d _J1;
  Eigen::Matrix2d _J2;
  Point2d* _p2d1;
  Point2d* _p2d2;

  LinearErr2(Point2d* p2d1, Point2d* p2d2) : _p2d1(p2d1), _p2d2(p2d2) {
    _p2d1->setActive(true);
    _p2d2->setActive(true);
    _J1.setRandom();
    _J2.setRandom();
    parent_t::setDesignVariables((aslam::backend::DesignVariable*)_p2d1, (aslam::backend::DesignVariable*)_p2d2);
    setInvR(sm::eigen::randomCovariance<2>());
    _p = _J1 * _p2d1->_v - _J2 * _p2d2->_v;
    _p[0] += sm::random::randn() * 0.1;
    _p[1] += sm::random::randn() * 0.1;
  }
  virtual ~LinearErr2() {}

  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    setError(_p - _J1 * _p2d1->_v - _J2 * _p2d2->_v);
    return evaluateChiSquaredError();
  }

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & J) const {
    J.add(_p2d1, -_J1);
    J.add(_p2d2, -_J2);
  }

};


class LinearErr3 : public aslam::backend::ErrorTermFs<4> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef aslam::backend::ErrorTermFs<4> parent_t;

  Eigen::Vector4d _p;
  Eigen::Matrix<double, 4, 2> _J1;
  Eigen::Matrix<double, 4, 2> _J2;
  Eigen::Matrix<double, 4, 2> _J3;
  Point2d* _p2d1;
  Point2d* _p2d2;
  Point2d* _p3;

  LinearErr3(Point2d* p2d1, Point2d* p2d2, Point2d* p3) : _p2d1(p2d1), _p2d2(p2d2), _p3(p3) {
    _p2d1->setActive(true);
    _p2d2->setActive(true);
    _p3->setActive(true);
    _J1.setRandom();
    _J2.setRandom();
    _J3.setRandom();
    parent_t::setDesignVariables((aslam::backend::DesignVariable*)_p2d1, (aslam::backend::DesignVariable*)_p2d2, (aslam::backend::DesignVariable*)_p3);
    setInvR(sm::eigen::randomCovariance<4>());
    _p =  _J1 * _p2d1->_v + _J2 * _p2d2->_v + _J3 * _p3->_v;
    _p[0] += sm::random::randn() * 0.1;
    _p[1] += sm::random::randn() * 0.1;
    _p[2] += sm::random::randn() * 0.1;
    _p[3] += sm::random::randn() * 0.1;
  }
  virtual ~LinearErr3() {}

  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    setError(_p - _J1 * _p2d1->_v - _J2 * _p2d2->_v - _J3 * _p3->_v);
    return evaluateChiSquaredError();
  }

  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & J) const {
    J.add(_p2d1, -_J1);
    J.add(_p2d2, -_J2);
    J.add(_p3, -_J3);
  }

};



inline void buildSystem(int D, int E, std::vector<aslam::backend::DesignVariable*>& dvs, std::vector<aslam::backend::ErrorTerm*>& errs)
{
  using namespace aslam::backend;
  int blockBase = 0;
  for (int i = 0; i < D; ++i) {
    dvs.push_back(new Point2d(Eigen::Vector2d::Random()));
    dvs.back()->setActive(true);
    dvs.back()->setBlockIndex(i);
    dvs.back()->setColumnBase(blockBase);
    blockBase += dvs.back()->minimalDimensions();
    //blocks.push_back(blockBase);
  }
  int rows = 0;
  for (int i = 0; i < E; ++i) {
    int mod = i % 3;
    if (mod == 0) {
      //std::cout << "le1\n";
      errs.push_back(new LinearErr((Point2d*)dvs[i % dvs.size()]));
    } else if (mod == 1) {
      //std::cout << "le2\n";
      errs.push_back(new LinearErr2((Point2d*)dvs[i % dvs.size()], (Point2d*)dvs[(i + 1) % dvs.size()]));
    } else {
      //std::cout << "le3\n";
      errs.push_back(new LinearErr3((Point2d*)dvs[i % dvs.size()], (Point2d*)dvs[(i + 1) % dvs.size()], (Point2d*)dvs[(i + 2) % dvs.size() ]));
    }
    errs.back()->setRowBase(rows);
    rows += errs.back()->dimension();
  }
}

inline void deleteSystem(std::vector<aslam::backend::DesignVariable*>& dvs, std::vector<aslam::backend::ErrorTerm*>& errs)
{
  using namespace aslam::backend;
  for (size_t i = 0; i < dvs.size(); ++i) {
    delete dvs[i];
  }
  dvs.clear();
  for (size_t i = 0; i < errs.size(); ++i) {
    delete errs[i];
  }
  errs.clear();
}


inline boost::shared_ptr<aslam::backend::OptimizationProblem> buildProblem(int seed, int D, int E)
{
  using namespace aslam::backend;
  srand(seed);
  sm::random::seed(seed);
  std::vector<aslam::backend::DesignVariable*> dvs;
  std::vector<aslam::backend::ErrorTerm*> errs;
  buildSystem(D, E, dvs, errs);
  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem);
  for (size_t i = 0; i < dvs.size(); ++i) {
    problem->addDesignVariable(dvs[i], true);
  }
  for (size_t i = 0; i < errs.size(); ++i) {
    problem->addErrorTerm(errs[i], true);
  }
  return problem;
}


#endif /* _SAMPLEDVANDERROR_H_ */
