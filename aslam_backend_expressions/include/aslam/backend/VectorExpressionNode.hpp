#ifndef ASLAM_BACKEND_VECTOR_EXPRESSION_NODE_HPP
#define ASLAM_BACKEND_VECTOR_EXPRESSION_NODE_HPP
#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/Differential.hpp>

namespace aslam {
  namespace backend {
    
    template<int D>
    class VectorExpressionNode
    {
    public:
      typedef Eigen::Matrix<double,D,1> vector_t;
      typedef vector_t value_t;
      typedef Differential<vector_t, double> differential_t;

      VectorExpressionNode() = default;
      virtual ~VectorExpressionNode() = default;
      
      vector_t evaluate() const { return evaluateImplementation(); }
      vector_t toVector() const { return evaluate(); }
      
      void evaluateJacobians(JacobianContainer & outJacobians) const;
      template <typename DERIVED>
      EIGEN_ALWAYS_INLINE void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixBase<DERIVED> & applyChainRule) const {
        evaluateJacobians(outJacobians.apply(applyChainRule));
      }

      void evaluateJacobians(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const;
      void getDesignVariables(DesignVariable::set_t & designVariables) const;

      virtual int getSize() const { assert(D != Eigen::Dynamic); return D; }
    private:
      virtual vector_t evaluateImplementation() const = 0;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;
      virtual void evaluateJacobiansImplementationWithDifferential(JacobianContainer & outJacobians, const differential_t & chainRuleDifferentail) const;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const = 0;
    };

    template <int D>
    class ConstantVectorExpressionNode : public VectorExpressionNode<D> {
     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef typename VectorExpressionNode<D>::vector_t vector_t;
      typedef typename VectorExpressionNode<D>::differential_t differential_t;

      ConstantVectorExpressionNode(int rows = D, int cols = 1) {
        if (D != Eigen::Dynamic){
          SM_ASSERT_EQ_DBG(std::runtime_error, rows, D, "dynamic size has to equal static size");
        }
        SM_ASSERT_EQ_DBG(std::runtime_error, cols, 1, "there is only one column supported as vector expression.");
      }
      ConstantVectorExpressionNode(const vector_t & value) : value(value) {}

      virtual ~ConstantVectorExpressionNode() = default;
      virtual int getSize() const override { return value.rows(); }
     private:
      virtual vector_t evaluateImplementation() const override { return value; }
      virtual void evaluateJacobiansImplementation(JacobianContainer &) const override {}
      virtual void getDesignVariablesImplementation(DesignVariable::set_t &) const override {}
     private:
      vector_t value;
    };
  } // namespace backend
} // namespace aslam


#include "implementation/VectorExpressionNode.hpp"

#endif /* ASLAM_BACKEND_VECTOR_EXPRESSION_NODE_HPP */
