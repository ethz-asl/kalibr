#ifndef ASLAM_BACKEND_VECTOR_EXPRESSION_HPP
#define ASLAM_BACKEND_VECTOR_EXPRESSION_HPP

#include <aslam/backend/JacobianContainer.hpp>
#include <boost/shared_ptr.hpp>
#include <sm/boost/null_deleter.hpp>
#include "VectorExpressionNode.hpp"
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/ScalarExpressionNode.hpp>

namespace aslam {
  namespace backend {
    

    template<int D>
    class VectorExpression
    {
    public:
      typedef Eigen::Matrix<double,D,1> vector_t;
      typedef Eigen::Matrix<double,D,1> value_t;

      VectorExpression(boost::shared_ptr< VectorExpressionNode<D> > root);
      VectorExpression(VectorExpressionNode<D> * root);
      virtual ~VectorExpression();
      
      vector_t evaluate() const;
      vector_t toValue() const;
      
      void evaluateJacobians(JacobianContainer & outJacobians) const;
      void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;

      void getDesignVariables(DesignVariable::set_t & designVariables) const;

      ScalarExpression toScalarExpression() const;

      boost::shared_ptr< VectorExpressionNode<D> > root() const { return _root; }

    private:
      boost::shared_ptr< VectorExpressionNode<D> > _root;
    };

  } // namespace backend
} // namespace aslam

#include "implementation/VectorExpression.hpp"

#endif /* ASLAM_BACKEND_VECTOR_EXPRESSION_HPP */
