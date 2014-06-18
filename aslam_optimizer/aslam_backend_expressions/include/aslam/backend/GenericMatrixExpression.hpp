#ifndef ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_HPP
#define ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_HPP

#include <set>
#include <type_traits>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <aslam/backend/JacobianContainer.hpp>
#include "ScalarExpression.hpp"
#include "GenericMatrixExpressionNode.hpp"
#include "Differential.hpp"

namespace aslam {
namespace backend {

namespace internal {
template<typename TNode>
struct GenericMatrixNodeTraits {
  typedef TNode node_t;
  typedef typename boost::shared_ptr<node_t> node_ptr_t;
  typedef typename node_t::matrix_t matrix_t;
  typedef typename node_t::value_t value_t;
  typedef typename node_t::tangent_vector_t tangent_vector_t;
  typedef typename node_t::differential_t differential_t;
  typedef typename node_t::constant_t constant_t;
};
}

template<int IRows, int ICols, typename TScalar = double, typename TNode = GenericMatrixExpressionNode<IRows, ICols, TScalar> >
class GenericMatrixExpression {
 public:
  typedef GenericMatrixExpression self_t;
  typedef GenericMatrixExpressionNode<IRows, ICols, TScalar> default_node_t;
  typedef TNode node_t;
  typedef GenericMatrixExpression<IRows, ICols, TScalar, default_node_t> default_self_t;

  typedef internal::GenericMatrixNodeTraits<node_t> node_traits_t;
  typedef typename node_traits_t::node_ptr_t node_ptr_t;
  typedef typename node_traits_t::matrix_t matrix_t;
  typedef typename node_traits_t::value_t value_t;
  typedef typename node_traits_t::tangent_vector_t tangent_vector_t;
  typedef typename node_traits_t::differential_t differential_t;
  typedef typename node_traits_t::constant_t constant_t;

  /// \brief initialize from an existing node.
  GenericMatrixExpression(node_ptr_t root);

  /// \brief Initialize from an existing node. The node will not be deleted.
  GenericMatrixExpression(node_t * root);

  /// \brief Initialize a constant expression from an matrix. The matrix will be copied.
  template<typename DERIVED>
  GenericMatrixExpression(const Eigen::MatrixBase<DERIVED> & mat);

  virtual ~GenericMatrixExpression() {
  }

  /// \brief Evaluate the full Eigen matrix.
  matrix_t toFullMatrix() const;

  /// \brief Evaluate the full Eigen matrix.
  matrix_t evaluate() const {
    return toFullMatrix();
  }

  /// \brief return the expression representing the Moore-Penrose-Inverse of this expression.
  GenericMatrixExpression<ICols, IRows, TScalar> inverse() const;

  /// \brief return the transposed matrix expression.
  GenericMatrixExpression<ICols, IRows, TScalar> transpose() const;

  template<int IColsOther, typename TOtherNode>
  GenericMatrixExpression<IRows, IColsOther, TScalar> operator*(const GenericMatrixExpression<ICols, IColsOther, TScalar, TOtherNode> & other) const;

  inline operator GenericMatrixExpression<IRows, ICols, TScalar>() const;

  default_self_t operator*(const ScalarExpression & scalarExpression) const;
  default_self_t operator*(TScalar scalar) const;
  default_self_t operator-() const;
  template<typename TOtherNode>
  default_self_t operator+(const GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> & other) const;
  template<typename TOtherNode>
  default_self_t operator-(const GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> & other) const;

  template<int IColsOther, typename TOtherNode, typename std::enable_if<IRows == 3 && (ICols == 1 || IColsOther == 1), int>::type = 0>
  GenericMatrixExpression<3, (ICols == 1 ? IColsOther : ICols), TScalar> cross(const GenericMatrixExpression<3, IColsOther, TScalar, TOtherNode> & other) const;

  /// \brief Evaluate the Jacobians
  void evaluateJacobians(JacobianContainer & outJacobians) const;
  void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
  void evaluateJacobians(JacobianContainer & outJacobians, const differential_t & diff) const;

  void getDesignVariables(DesignVariable::set_t & designVariables) const;

  inline node_ptr_t root() const {
    return _root;
  }

  inline static self_t constant(int rows = IRows, int cols = ICols) {
    return GenericMatrixExpression(new constant_t(rows, cols));
  }
 protected:
  GenericMatrixExpression() {
  }
 private:
  node_ptr_t _root;

 public:
  template<typename TDerived, typename TOperand>
  class UnaryOperationResult : public node_t {
   public:
    typedef TOperand operand_t;
    typedef typename TOperand::node_t operand_node_t;
    typedef typename node_t::differential_t::domain_t apply_diff_return_t;

    virtual ~UnaryOperationResult() {
    }

    inline apply_diff_return_t applyDiff(const typename operand_t::tangent_vector_t & /* tangent_vector */) const {
      throw std::runtime_error("This method must be shadowed or not used!");
    }

    static inline self_t create(const TOperand & operand) {
      TDerived * p = new TDerived();
      p->_operand = operand.root();
      return self_t(node_ptr_t(p));
    }
    template<typename TData>
    static inline self_t create(const TOperand & operand, TData data) {
      TDerived * p = new TDerived(data);
      p->_operand = operand.root();
      return self_t(node_ptr_t(p));
    }

   protected:
    inline UnaryOperationResult() {
    }
    const operand_node_t& getOperandNode() const {
      return *_operand;
    }
    virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
      _operand->getDesignVariables(designVariables);
    }
    virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename node_t::differential_t & chainRuleDifferentail) const {
      this->getOperandNode().evaluateJacobians(outJacobians, Diff(*static_cast<const TDerived *>(this), chainRuleDifferentail));
    }
   private:
    typename TOperand::node_ptr_t _operand;

    class Diff : public ComposedDifferential<typename operand_t::tangent_vector_t, apply_diff_return_t, TScalar, Diff> {
      const TDerived & _derivedNode;
     public:
      typedef ComposedDifferential<typename operand_t::tangent_vector_t, apply_diff_return_t, TScalar, Diff> base_t;
      typedef typename base_t::domain_t domain_t;

      inline Diff(const TDerived & derivedNode, const typename TDerived::differential_t & diff)
          : base_t(diff),
            _derivedNode(derivedNode) {
      }
      ~Diff() {
      }

      inline apply_diff_return_t apply(const typename operand_t::tangent_vector_t & tangent_vector) const {
        return _derivedNode.applyDiff(tangent_vector);
      }
    };
  };

  template<typename TDerived, typename TLhs, typename TRhs>
  class BinaryOperationResult : public node_t {
   public:
    typedef typename node_t::differential_t::domain_t apply_diff_return_t;

    virtual ~BinaryOperationResult() {
    }

    typedef TLhs lhs_t;
    typedef TRhs rhs_t;
    typedef typename TLhs::node_t lhs_node_t;
    typedef typename TRhs::node_t rhs_node_t;

    inline apply_diff_return_t applyLhsDiff(const typename lhs_t::tangent_vector_t & tangent_vector) const {
      throw std::runtime_error("This method must be shadowed or not used!");
    }
    inline apply_diff_return_t applyRhsDiff(const typename rhs_t::tangent_vector_t & tangent_vector) const {
      throw std::runtime_error("This method must be shadowed or not used!");
    }

    static inline self_t create(const TLhs & l, const TRhs & r) {
      TDerived * p = new TDerived();
      p->_lhs = l.root();
      p->_rhs = r.root();
      return self_t(node_ptr_t(p));
    }
   protected:
    inline BinaryOperationResult() {
    }
    const lhs_node_t & getLhsNode() const {
      return *_lhs;
    }
    const rhs_node_t & getRhsNode() const {
      return *_rhs;
    }
    virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }
    virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename node_t::differential_t & diff) const {
      this->getLhsNode().evaluateJacobians(outJacobians, Diff<lhs_t, &TDerived::applyLhsDiff>(*static_cast<const TDerived *>(this), diff));
      this->getRhsNode().evaluateJacobians(outJacobians, Diff<rhs_t, &TDerived::applyRhsDiff>(*static_cast<const TDerived *>(this), diff));
    }
   private:
    typename TLhs::node_ptr_t _lhs;
    typename TRhs::node_ptr_t _rhs;

    template<typename TSide, apply_diff_return_t (TDerived::*FApply)(const typename TSide::tangent_vector_t &) const>
    class Diff : public ComposedDifferential<typename TSide::tangent_vector_t, apply_diff_return_t, TScalar, Diff<TSide, FApply> > {
      const TDerived & _derivedNode;
     public:
      typedef ComposedDifferential<typename TSide::tangent_vector_t, apply_diff_return_t, TScalar, Diff> base_t;
      typedef typename base_t::domain_t domain_t;

      inline Diff(const TDerived & derivedNode, const typename TDerived::differential_t & diff)
          : base_t(diff),
            _derivedNode(derivedNode) {
      }
      ~Diff() {
      }

      inline apply_diff_return_t apply(const typename TSide::tangent_vector_t & tangent_vector) const {
        return (_derivedNode.*FApply)(tangent_vector);
      }
    };
  };
};

}  // namespace backend
}  // namespace aslam

#include "implementation/GenericMatrixExpression.hpp"

#endif /* ASLAM_BACKEND_MATRIX_EXPRESSION_HPP */
