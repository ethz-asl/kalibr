#include <sm/eigen/gtest.hpp>

#include <aslam/backend/test/ErrorTermTester.hpp>
#include "SampleDvAndError.hpp"

class BadErr : public LinearErr {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BadErr(Point2d* p2d) : LinearErr(p2d) {}
  virtual ~BadErr() {}

  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation() {
    setError(Eigen::Vector2d::Zero());
    return evaluateChiSquaredError();
  }
};

TEST(ErrorTermTester, testGood)
{
  Eigen::Vector2d v;
  Point2d p(v);
  LinearErr error_term(&p);

  SCOPED_TRACE("");
  testErrorTerm(error_term);
}

TEST(ErrorTermTester, testFailing)
{
  Eigen::Vector2d v;
  Point2d p(v);
  BadErr error_term(&p);

  SCOPED_TRACE("");
  testErrorTerm(error_term);
}

