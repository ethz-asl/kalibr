#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/GenericScalarExpressionNode.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
namespace backend {

template<typename Scalar_>
Scalar_ GenericScalarExpression<Scalar_>::toScalar() const {
  if(empty()) return Scalar_(0.0);
  return this->_root->toScalar();
}

template<typename Scalar_>
void GenericScalarExpression<Scalar_>::evaluateJacobians(JacobianContainer & outJacobians) const{
  if(empty()) return;
  _root->evaluateJacobians(outJacobians);
}
template<typename Scalar_>
void GenericScalarExpression<Scalar_>::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const{
  if(empty()) return;
  _root->evaluateJacobians(outJacobians, applyChainRule);
}
template<typename Scalar_>
void GenericScalarExpression<Scalar_>::getDesignVariables(DesignVariable::set_t & designVariables) const {
  if(empty()) return;
  _root->getDesignVariables(designVariables);
}

namespace internal {
template<typename Scalar_, typename NodeType_>
class GSEUnRes : public GenericScalarExpressionNode<Scalar_> {
 public:
  typedef GenericScalarExpressionNode<Scalar_> Base;
  typedef boost::shared_ptr<NodeType_> SharedNodePointer;
  GSEUnRes(SharedNodePointer lhs) : _lhs(lhs) {}
  virtual ~GSEUnRes() {}
 protected:
  void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    this->_lhs->getDesignVariables(designVariables);
  }
  SharedNodePointer _lhs;
};

template<typename Scalar_>
class GSEBinRes : public GenericScalarExpressionNode<Scalar_> {
 public:
  typedef GenericScalarExpressionNode<Scalar_> Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;

  GSEBinRes(SharedNodePointer lhs, SharedNodePointer rhs) : _lhs(lhs), _rhs(rhs) {}
  virtual ~GSEBinRes() {}
 protected:
  void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    this->_lhs->getDesignVariables(designVariables);
    this->_rhs->getDesignVariables(designVariables);
  }

  SharedNodePointer _lhs;
  SharedNodePointer _rhs;
};



template<typename Scalar_, typename OtherScalar_>
class GenericScalarExpressionNodeCasted : public GSEUnRes<Scalar_, GenericScalarExpressionNode<OtherScalar_> > {
 public:
  typedef GSEUnRes<Scalar_, GenericScalarExpressionNode<OtherScalar_> > Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeCasted(SharedNodePointer lhs) : Base(lhs) {}
  virtual ~GenericScalarExpressionNodeCasted() {}
 protected:

  Scalar_ evaluateImplementation() const {
    return Scalar_(this->_lhs->toScalar());
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians);
  }
};


template<typename Scalar_>
class GenericScalarExpressionNodeConstant : public GenericScalarExpressionNode<Scalar_> {
 public:
  typedef GenericScalarExpressionNode<Scalar_> Base;
  typedef Scalar_ Scalar;
  GenericScalarExpressionNodeConstant(Scalar s) : _s(s) {}
  virtual ~GenericScalarExpressionNodeConstant() {}
 protected:
  virtual Scalar evaluateImplementation() const override { return _s; }
  virtual void evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */) const override {}
  virtual void getDesignVariablesImplementation(DesignVariable::set_t & /* designVariables */) const override {}

  Scalar _s;
};

template<typename Scalar_>
class GenericScalarExpressionNodeMultiply : public GSEBinRes<Scalar_> {
 public:
  typedef GSEBinRes<Scalar_> Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeMultiply(SharedNodePointer lhs, SharedNodePointer rhs) : Base(lhs, rhs) {}
  virtual ~GenericScalarExpressionNodeMultiply(){}
 protected:
  Scalar_ evaluateImplementation() const {
    return this->_lhs->toScalar() * this->_rhs->toScalar();
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians.apply(this->_rhs->toScalar()));
    this->_rhs->evaluateJacobians(outJacobians.apply(this->_lhs->toScalar()));
  }
};

template<typename Scalar_>
class GenericScalarExpressionNodeDivide : public GSEBinRes<Scalar_> {
 public:
  typedef GSEBinRes<Scalar_> Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeDivide(SharedNodePointer lhs, SharedNodePointer rhs) : Base(lhs, rhs) {}
  virtual ~GenericScalarExpressionNodeDivide() {}
 protected:

  Scalar_ evaluateImplementation() const {
    return this->_lhs->toScalar() / this->_rhs->toScalar();
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians.apply(-this->_lhs->toScalar() / (this->_rhs->toScalar() * this->_rhs->toScalar())));
    this->_rhs->evaluateJacobians(outJacobians.apply(1.0 / this->_rhs->toScalar()));
  }
};

template<typename Scalar_>
class GenericScalarExpressionNodeAdd : public GSEBinRes<Scalar_> {
 public:
  typedef GSEBinRes<Scalar_> Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeAdd(SharedNodePointer lhs, SharedNodePointer rhs) : Base(lhs, rhs) {}
  virtual ~GenericScalarExpressionNodeAdd() {}
 protected:

  Scalar_ evaluateImplementation() const {
    return this->_lhs->toScalar() + this->_rhs->toScalar();
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians);
    this->_rhs->evaluateJacobians(outJacobians);
  }
};

template<typename Scalar_>
class GenericScalarExpressionNodeSub : public GSEBinRes<Scalar_> {
 public:
  typedef GSEBinRes<Scalar_> Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeSub(SharedNodePointer lhs, SharedNodePointer rhs) : Base(lhs, rhs) {}
  virtual ~GenericScalarExpressionNodeSub() {}
 protected:

  Scalar_ evaluateImplementation() const {
    return this->_lhs->toScalar() - this->_rhs->toScalar();
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians);
    this->_rhs->evaluateJacobians(outJacobians.apply(-1.0));
  }
};

template<typename Scalar_>
class GenericScalarExpressionNodeNegated : public GSEUnRes<Scalar_, GenericScalarExpressionNode<Scalar_> > {
 public:
  typedef GSEUnRes<Scalar_, GenericScalarExpressionNode<Scalar_> > Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeNegated(SharedNodePointer lhs) : Base(lhs) {}
  virtual ~GenericScalarExpressionNodeNegated() {}
 protected:

  Scalar_ evaluateImplementation() const {
    return -this->_lhs->toScalar();
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians.apply(-1.0));
  }
};

template<typename Scalar_>
class GenericScalarExpressionNodeFromVectorExpression : public GSEUnRes<Scalar_, VectorExpressionNode<1> > {
 public:
  typedef GSEUnRes<Scalar_, VectorExpressionNode<1> > Base;
  typedef typename Base::SharedNodePointer SharedNodePointer;
  GenericScalarExpressionNodeFromVectorExpression(SharedNodePointer lhs) : Base(lhs) {}
  virtual ~GenericScalarExpressionNodeFromVectorExpression();

 protected:
  // These functions must be implemented by child classes.
  double evaluateImplementation() const {
    return this->_lhs->evaluate()(0);
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    this->_lhs->evaluateJacobians(outJacobians);
  }
};
}  // namepsace internal

template<typename Scalar_>
GenericScalarExpression<Scalar_>::GenericScalarExpression(Scalar_ value) : _root(new internal::GenericScalarExpressionNodeConstant<Scalar_>(value)) {
}

template <typename Scalar_>
template <typename OtherScalar, typename>
inline GenericScalarExpression<Scalar_>::GenericScalarExpression(const GenericScalarExpression<OtherScalar> & other) : _root(new internal::GenericScalarExpressionNodeCasted<Scalar_, OtherScalar>(other._root)){
}


template<typename Scalar_>
GenericScalarExpression<Scalar_>::GenericScalarExpression(NodeType * node, bool expressionOwnsNode) :
  _root(expressionOwnsNode ? SharedNodePointer(node) : SharedNodePointer(node, sm::null_deleter())){
}

template<typename Scalar_>
GenericScalarExpression<Scalar_> GenericScalarExpression<Scalar_>::operator+(const GenericScalarExpression & s) {
  return SharedNodePointer(new internal::GenericScalarExpressionNodeAdd<Scalar_>(_root, s._root));
}

template<typename Scalar_>
GenericScalarExpression<Scalar_> GenericScalarExpression<Scalar_>::operator-() {
  return SharedNodePointer(new internal::GenericScalarExpressionNodeNegated<Scalar_>(_root));
}

template<typename Scalar_>
GenericScalarExpression<Scalar_> GenericScalarExpression<Scalar_>::operator-(const GenericScalarExpression & s) {
  return SharedNodePointer(new internal::GenericScalarExpressionNodeSub<Scalar_>(_root, s._root));
}

template<typename Scalar_>
GenericScalarExpression<Scalar_> GenericScalarExpression<Scalar_>::operator/(const GenericScalarExpression & s) {
  return SharedNodePointer(new internal::GenericScalarExpressionNodeDivide<Scalar_>(_root, s._root));
}

template<typename Scalar_>
GenericScalarExpression<Scalar_> GenericScalarExpression<Scalar_>::operator*(const GenericScalarExpression & s) {
  return SharedNodePointer(new internal::GenericScalarExpressionNodeMultiply<Scalar_>(_root, s._root));
}

}  // namespace backend
}  // namespace aslam
