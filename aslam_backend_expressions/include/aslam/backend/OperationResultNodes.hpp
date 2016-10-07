#ifndef ASLAM_BACKEND_OPERATION_RESULT_NODES_HPP
#define ASLAM_BACKEND_OPERATION_RESULT_NODES_HPP

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/Differential.hpp>

namespace aslam {
namespace backend {
  class ScalarExpressionNode;

namespace internal {
  template <typename Node>
  struct NodeTraits {
    typedef typename Node::differential_t differential_t;
    typedef boost::shared_ptr<Node> node_ptr_t;
    typedef typename differential_t::domain_t tangent_vector_t;
  };

  template <>
  struct NodeTraits<ScalarExpressionNode> {
    typedef Differential<Eigen::Matrix<double, 1, 1>, double> differential_t;
    typedef boost::shared_ptr<ScalarExpressionNode> node_ptr_t;
    typedef typename differential_t::domain_t tangent_vector_t;
  };
}

template<typename TDerived, typename TOperand, typename TResult, typename TApplyDiffReturn, typename TScalar>
class UnaryOperationResultNode : public TResult::node_t {
 public:
  typedef TOperand operand_t;
  typedef TResult self_t;
  typedef typename TResult::node_t node_t;
  typedef internal::NodeTraits<node_t> node_traits_t;
  typedef typename TOperand::node_t operand_node_t;
  typedef TApplyDiffReturn apply_diff_return_t;
  typedef internal::NodeTraits<operand_node_t> operand_node_traits_t;

  virtual ~UnaryOperationResultNode() {
  }

  inline apply_diff_return_t applyDiff(const typename operand_node_traits_t::tangent_vector_t & /* tangent_vector */) const {
    throw std::runtime_error("This method must be shadowed or not used!");
  }

  static inline self_t create(const TOperand & operand) {
    TDerived * p = new TDerived();
    p->_operand = operand.root();
    return self_t(typename node_traits_t::node_ptr_t(p));
  }
  template<typename TData>
  static inline self_t create(const TOperand & operand, TData data) {
    TDerived * p = new TDerived(data);
    p->_operand = operand.root();
    return self_t(typename node_traits_t::node_ptr_t(p));
  }

 protected:
  inline UnaryOperationResultNode() {
  }
  const operand_node_t& getOperandNode() const {
    return *_operand;
  }
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    _operand->getDesignVariables(designVariables);
  }
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename node_traits_t::differential_t & chainRuleDifferentail) const {
    this->getOperandNode().evaluateJacobians(outJacobians, Diff(*static_cast<const TDerived *>(this), chainRuleDifferentail));
  }
 private:
  typename operand_node_traits_t::node_ptr_t _operand;

  class Diff : public ComposedDifferential<typename operand_node_traits_t::tangent_vector_t, apply_diff_return_t, TScalar, Diff> {
    const TDerived & _derivedNode;
   public:
    typedef ComposedDifferential<typename operand_node_traits_t::tangent_vector_t, apply_diff_return_t, TScalar, Diff> base_t;
    typedef typename base_t::domain_t domain_t;

    inline Diff(const TDerived & derivedNode, const typename node_traits_t::differential_t & diff)
        : base_t(diff),
          _derivedNode(derivedNode) {
    }
    ~Diff() {
    }

    inline apply_diff_return_t apply(const typename operand_node_traits_t::tangent_vector_t & tangent_vector) const {
      return _derivedNode.applyDiff(tangent_vector);
    }
  };
};



template<typename TDerived, typename TLhs, typename TRhs, typename TResult, typename TApplyDiffReturn, typename TScalar>
class BinaryOperationResultNode : public TResult::node_t {
 public:
  typedef TApplyDiffReturn apply_diff_return_t;
  typedef TResult self_t;
  typedef typename TResult::node_t node_t;
  typedef internal::NodeTraits<node_t> node_traits_t;


  virtual ~BinaryOperationResultNode() {
  }

  typedef TLhs lhs_t;
  typedef TRhs rhs_t;
  typedef typename TLhs::node_t lhs_node_t;
  typedef typename TRhs::node_t rhs_node_t;
  typedef internal::NodeTraits<lhs_node_t> lhs_node_traits_t;
  typedef internal::NodeTraits<rhs_node_t> rhs_node_traits_t;

  inline apply_diff_return_t applyLhsDiff(const typename lhs_node_traits_t::tangent_vector_t & tangent_vector) const {
    throw std::runtime_error("This method must be shadowed or not used!");
  }
  inline apply_diff_return_t applyRhsDiff(const typename rhs_node_traits_t::tangent_vector_t & tangent_vector) const {
    throw std::runtime_error("This method must be shadowed or not used!");
  }

  static inline self_t create(const TLhs & l, const TRhs & r) {
    TDerived * p = new TDerived();
    p->_lhs = l.root();
    p->_rhs = r.root();
    return self_t(typename node_traits_t::node_ptr_t(p));
  }
 protected:
  inline BinaryOperationResultNode() {
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
  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename node_traits_t::differential_t & diff) const {
    this->getLhsNode().evaluateJacobians(outJacobians, Diff<lhs_t, &TDerived::applyLhsDiff>(*static_cast<const TDerived *>(this), diff));
    this->getRhsNode().evaluateJacobians(outJacobians, Diff<rhs_t, &TDerived::applyRhsDiff>(*static_cast<const TDerived *>(this), diff));
  }
 private:
  typename lhs_node_traits_t::node_ptr_t _lhs;
  typename rhs_node_traits_t::node_ptr_t _rhs;

  template<typename TSide, apply_diff_return_t (TDerived::*FApply)(const typename internal::NodeTraits<typename TSide::node_t>::tangent_vector_t &) const>
  class Diff : public ComposedDifferential<typename internal::NodeTraits<typename TSide::node_t>::tangent_vector_t, apply_diff_return_t, TScalar, Diff<TSide, FApply> > {
    const TDerived & _derivedNode;
   public:
    typedef typename TSide::node_t side_node_t;
    typedef typename internal::NodeTraits<side_node_t> side_node_traits_t;

    typedef ComposedDifferential<typename side_node_traits_t::tangent_vector_t, apply_diff_return_t, TScalar, Diff> base_t;
    typedef typename base_t::domain_t domain_t;

    inline Diff(const TDerived & derivedNode, const typename TDerived::differential_t & diff)
        : base_t(diff),
          _derivedNode(derivedNode) {
    }
    ~Diff() {
    }

    inline apply_diff_return_t apply(const typename side_node_traits_t::tangent_vector_t & tangent_vector) const {
      return (_derivedNode.*FApply)(tangent_vector);
    }
  };
};

}  // namespace backend
}  // namespace aslam
#endif
