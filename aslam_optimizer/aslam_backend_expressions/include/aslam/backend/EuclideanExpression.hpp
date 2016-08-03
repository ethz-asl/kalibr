#ifndef ASLAM_EUCLIDEAN_EXPRESSION_HPP
#define ASLAM_EUCLIDEAN_EXPRESSION_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <set>

namespace aslam {
  namespace backend {
    class HomogeneousExpression;
    class EuclideanExpressionNode;
    class ScalarExpression;
    template <int D> class VectorExpression;
    
    class EuclideanExpression
    {
    public:
      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      typedef Eigen::Vector3d value_t;
      typedef Eigen::Vector3d vector_t;

      EuclideanExpression(EuclideanExpressionNode * designVariable);
      EuclideanExpression(boost::shared_ptr<EuclideanExpressionNode> designVariable);
      EuclideanExpression(const VectorExpression<3> & vectorExpression);
      // Create a constant expression
      EuclideanExpression(const Eigen::Vector3d & p);

      virtual ~EuclideanExpression();
      
      value_t toEuclidean() const;
      value_t evaluate() const { return toEuclidean(); }
      value_t toValue() const { return toEuclidean(); }
      HomogeneousExpression toHomogeneousExpression() const;

      void evaluateJacobians(JacobianContainer & outJacobians) const;
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;

      EuclideanExpression cross(const EuclideanExpression & p) const;
      EuclideanExpression operator+(const EuclideanExpression & p) const;
      EuclideanExpression operator-(const EuclideanExpression & p) const;
      EuclideanExpression operator-(const Eigen::Vector3d & p) const;
      EuclideanExpression operator-() const;
      EuclideanExpression operator*(const ScalarExpression& s) const;
      EuclideanExpression elementwiseMultiply(const EuclideanExpression & p) const;

      void getDesignVariables(DesignVariable::set_t & designVariables) const;

      boost::shared_ptr<EuclideanExpressionNode> root() { return _root; }
    private:
      /// \todo make the default constructor private.
      EuclideanExpression();

      friend class RotationExpression;
      friend class MatrixExpression;
      
      boost::shared_ptr<EuclideanExpressionNode> _root;
    };
    
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_EUCLIDEAN_EXPRESSION_HPP */
