#ifndef ASLAM_EUCLIDEAN_EXPRESSION_HPP
#define ASLAM_EUCLIDEAN_EXPRESSION_HPP

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <set>

#include "VectorExpression.hpp"

namespace aslam {
  namespace backend {
    class HomogeneousExpression;
    template <int D> class VectorExpressionNode;
    typedef VectorExpressionNode<3> EuclideanExpressionNode;
    class ScalarExpression;
    
    class EuclideanExpression : public VectorExpression<3>
    {
    public:
      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      typedef Eigen::Vector3d value_t;
      typedef Eigen::Vector3d vector_t;

      EuclideanExpression() = default;
      EuclideanExpression(EuclideanExpressionNode * root) : VectorExpression(root) {}
      EuclideanExpression(boost::shared_ptr<EuclideanExpressionNode> root) : VectorExpression<3>(root) {}
      EuclideanExpression(const VectorExpression<3> & vectorExpression) : VectorExpression<3>(vectorExpression){}

      // Create a constant expression
      EuclideanExpression(const Eigen::Vector3d & p) : VectorExpression<3>(p) {}

      value_t toEuclidean() const { return evaluate(); }
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

    private:
      friend class RotationExpression;
      friend class MatrixExpression;
    };
    
  } // namespace backend
} // namespace aslam


#endif /* ASLAM_EUCLIDEAN_EXPRESSION_HPP */
