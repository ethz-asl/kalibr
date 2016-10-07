#ifndef ASLAM_SCALAR_EXPRESSION_HPP
#define ASLAM_SCALAR_EXPRESSION_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/Differential.hpp>
#include <set>

namespace aslam {
  namespace backend {

    using std::sqrt;
    using std::log;
    using std::exp;

    class ScalarExpressionNode;
    
    class ScalarExpression
    {
    public:
      enum { Dimension = 1 };
      typedef double Value;
      typedef double value_t;
      typedef ScalarExpressionNode node_t;

      explicit ScalarExpression( double value );
      ScalarExpression(ScalarExpressionNode * designVariable);
      ScalarExpression(boost::shared_ptr<ScalarExpressionNode> designVariable);
      ~ScalarExpression();
      
      double toScalar() const;
      double toValue() const { return toScalar(); }
      double evaluate() const { return toScalar(); }

      void evaluateJacobians(JacobianContainer & outJacobians) const;
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<ScalarExpressionNode> root() const { return _root; }
      bool isEmpty() const { return false; }  //TODO feature: support empty scalar expression

      ScalarExpression operator+(const ScalarExpression & s) const;
      ScalarExpression operator-(const ScalarExpression & s) const;
      ScalarExpression operator*(const ScalarExpression & s) const;
      ScalarExpression operator/(const ScalarExpression & s) const;
      ScalarExpression operator+(double s) const;
      ScalarExpression operator-(double s) const;
      ScalarExpression operator-() const;
      ScalarExpression operator*(double s) const;
      ScalarExpression operator/(double s) const;
      bool operator < (const ScalarExpression & s) const { return this->toValue() < s.toValue(); }
      bool operator > (const ScalarExpression & s) const { return this->toValue() > s.toValue(); }
      bool operator <= (const ScalarExpression & s) const { return this->toValue() <= s.toValue(); }
      bool operator >= (const ScalarExpression & s) const { return this->toValue() >= s.toValue(); }
      bool operator == (const ScalarExpression & s) const { return this->toValue() == s.toValue(); }
      bool operator != (const ScalarExpression & s) const { return this->toValue() != s.toValue(); }
      bool operator < (const double s) const { return this->toValue() < s; }
      bool operator > (const double s) const { return this->toValue() > s; }
      bool operator <= (const double s) const { return this->toValue() <= s; }
      bool operator >= (const double s) const { return this->toValue() >= s; }
      bool operator == (const double s) const { return this->toValue() == s; }
      bool operator != (const double s) const { return this->toValue() != s; }

    private:
      /// \todo make the default constructor private.
      ScalarExpression();

      boost::shared_ptr<ScalarExpressionNode> _root;

      friend class EuclideanExpression;

    };
    
    std::ostream& operator<<(std::ostream& os, const ScalarExpression& e);

    ScalarExpression sqrt(const ScalarExpression& e);
    ScalarExpression log(const ScalarExpression& e);
    ScalarExpression exp(const ScalarExpression& e);
    ScalarExpression atan(const ScalarExpression& e);
    ScalarExpression tanh(const ScalarExpression& e);
    ScalarExpression atan2(const ScalarExpression& e0, const ScalarExpression& e1);
    ScalarExpression acos(const ScalarExpression& e);
    ScalarExpression acosSquared(const ScalarExpression& e);
    ScalarExpression inverseSigmoid(const ScalarExpression& e, const double height, const double scale, const double shift);
    ScalarExpression powerExpression(const ScalarExpression& e, const int k);
    ScalarExpression piecewiseExpression(const ScalarExpression& e1, const ScalarExpression& e2, std::function<bool()> useFirst);
    inline ScalarExpression operator / (const double num, const ScalarExpression& den) {
      return ScalarExpression(num) / den;
    }

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_SCALAR_EXPRESSION_HPP */
