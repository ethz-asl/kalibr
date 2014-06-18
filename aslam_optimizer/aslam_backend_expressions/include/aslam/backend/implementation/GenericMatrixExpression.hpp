#include <sm/boost/null_deleter.hpp>
#include <Eigen/Geometry>

namespace aslam {
namespace backend {

#define _TEMPLATE template <int IRows, int ICols, typename TScalar, typename TNode>
#define _CLASS GenericMatrixExpression<IRows, ICols, TScalar, TNode>

_TEMPLATE _CLASS::GenericMatrixExpression(node_ptr_t root) :
_root(root)
{
}

_TEMPLATE
_CLASS::GenericMatrixExpression(node_t * root) :
_root(root, sm::null_deleter() )
{
}

_TEMPLATE
template <typename DERIVED>
_CLASS::GenericMatrixExpression(const Eigen::MatrixBase<DERIVED> & mat) : _root(new typename node_t::constant_t(mat))
{
}

_TEMPLATE
typename _CLASS::matrix_t _CLASS::toFullMatrix() const
{
  return _root->evaluate();
}

_TEMPLATE
void _CLASS::evaluateJacobians(JacobianContainer & outJacobians) const
{
  _root->evaluateJacobians(outJacobians, IdentityDifferential<tangent_vector_t, TScalar>());
}

_TEMPLATE
void _CLASS::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
{
  _root->evaluateJacobians(outJacobians, MatrixDifferential<TScalar, Eigen::Matrix<TScalar, Eigen::Dynamic, IRows> >(applyChainRule.cast<TScalar>()));
}

_TEMPLATE
void _CLASS::evaluateJacobians(JacobianContainer & outJacobians, const differential_t & diff) const
{
  return _root->evaluateJacobians(outJacobians, diff);
}

_TEMPLATE
void _CLASS::getDesignVariables(DesignVariable::set_t & designVariables) const
{
  return _root->getDesignVariables(designVariables);
}

_TEMPLATE
inline _CLASS::operator GenericMatrixExpression<IRows, ICols,TScalar>()const{typedef default_self_t result_t;

class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
public:
  typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

  virtual ~ResultNode() {}

  virtual void evaluateImplementation() const {
    this->_currentValue = this->getOperandNode().evaluate();
  }

  inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangent_vector) const {
    return tangent_vector;
  }
};
return ResultNode::create(*this);
}

_TEMPLATE GenericMatrixExpression<ICols, IRows, TScalar> _CLASS::transpose() const {
  typedef GenericMatrixExpression<ICols, IRows, TScalar> result_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = this->getOperandNode().evaluate().transpose();
    }

    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangent_vector) const {
      return tangent_vector.transpose();
    }
  };

  return ResultNode::create(*this);
}

_TEMPLATE GenericMatrixExpression<ICols, IRows, TScalar> _CLASS::inverse() const {
  typedef GenericMatrixExpression<ICols, IRows, TScalar> result_t;

  static_assert(ICols == IRows, "generalized inverse not yet supported.");

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = this->getOperandNode().evaluate().inverse();
    }

    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangent_vector) const {
      evaluateImplementation();
      return - this->_currentValue * tangent_vector * this->_currentValue;
    }
  };

  return ResultNode::create(*this);
}

_TEMPLATE template <int IColsOther, typename TOtherNode>
GenericMatrixExpression<IRows, IColsOther, TScalar> _CLASS::operator*(const GenericMatrixExpression<ICols, IColsOther, TScalar, TOtherNode> & other) const {
  typedef GenericMatrixExpression<IRows, IColsOther, TScalar> result_t;
  typedef GenericMatrixExpression<ICols, IColsOther, TScalar, TOtherNode> other_t;

  class ResultNode : public result_t::template BinaryOperationResult<ResultNode, self_t, other_t> {
  public:
    typedef typename result_t::template BinaryOperationResult<ResultNode, self_t, other_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = this->getLhsNode().evaluate() * this->getRhsNode().evaluate();
    }

    inline typename base_t::apply_diff_return_t applyLhsDiff(const typename base_t::lhs_t::tangent_vector_t & tangent_vector) const {
      return tangent_vector * this->getRhsNode().evaluate();
    }
    inline typename base_t::apply_diff_return_t applyRhsDiff(const typename base_t::rhs_t::tangent_vector_t & tangent_vector) const {
      return this->getLhsNode().evaluate() * tangent_vector;
    }
  };

  return ResultNode::create(*this, other);
}

_TEMPLATE template<typename TOtherNode>
typename _CLASS::default_self_t _CLASS::operator+(const GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> & other) const {
  typedef default_self_t result_t;
  typedef GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> other_t;

  class ResultNode : public result_t::template BinaryOperationResult<ResultNode, self_t, other_t> {
  public:
    typedef typename result_t::template BinaryOperationResult<ResultNode, self_t, other_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = this->getLhsNode().evaluate() + this->getRhsNode().evaluate();
    }

    inline typename base_t::apply_diff_return_t applyLhsDiff(const typename base_t::lhs_t::tangent_vector_t & tangent_vector) const {
      return tangent_vector;
    }
    inline typename base_t::apply_diff_return_t applyRhsDiff(const typename base_t::rhs_t::tangent_vector_t & tangent_vector) const {
      return tangent_vector;
    }
  };

  return ResultNode::create(*this, other);
}

_TEMPLATE
template<typename TOtherNode>
typename _CLASS::default_self_t _CLASS::operator-(const GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> & other) const {
  typedef default_self_t result_t;
  typedef GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> other_t;

  class ResultNode : public result_t::template BinaryOperationResult<ResultNode, self_t, other_t> {
  public:
    typedef typename result_t::template BinaryOperationResult<ResultNode, self_t, other_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = this->getLhsNode().evaluate() - this->getRhsNode().evaluate();
    }

    inline typename base_t::apply_diff_return_t applyLhsDiff(const typename base_t::lhs_t::tangent_vector_t & tangent_vector) const {
      return tangent_vector;
    }
    inline typename base_t::apply_diff_return_t applyRhsDiff(const typename base_t::rhs_t::tangent_vector_t & tangent_vector) const {
      return -tangent_vector;
    }
  };

  return ResultNode::create(*this, other);
}

_TEMPLATE
typename _CLASS::default_self_t _CLASS::operator-() const {
  typedef default_self_t result_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = -this->getOperandNode().evaluate();
    }

    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangent_vector) const {
      return -tangent_vector;
    }
  };

  return ResultNode::create(*this);
}

_TEMPLATE
typename _CLASS::default_self_t _CLASS::operator*(TScalar scalar) const {
  typedef default_self_t result_t;

  class ResultNode : public result_t::template UnaryOperationResult<ResultNode, self_t> {
  public:
    typedef typename result_t::template UnaryOperationResult<ResultNode, self_t> base_t;

    ResultNode(TScalar scalar) : _scalar(scalar) {}
    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      this->_currentValue = this->getOperandNode().evaluate() * _scalar;
    }

    inline typename base_t::apply_diff_return_t applyDiff(const typename base_t::operand_t::tangent_vector_t & tangent_vector) const {
      return tangent_vector * _scalar;
    }
  private:
    TScalar _scalar;
  };

  return ResultNode::create(*this, scalar);
}

namespace internal {
  template <int ILhsCols, int IRhsCols>
  struct CrossProductCalculator {
  };

  template <>
  struct CrossProductCalculator<Eigen::Dynamic, Eigen::Dynamic> {
    template <typename TLhs, typename TRhs, typename TResult>
    inline static void calcCrossInto(const TLhs & lhs, const TRhs & rhs, TResult & result) {
      const int resultCols = result.cols();
      if(lhs.cols() == 1)
      for(int i = 0; i < resultCols; i++) result.template block<3,1>(0, i) = lhs.cross(rhs.template block<3, 1>(0, i));
      else
      for(int i = 0; i < resultCols; i++) result.template block<3,1>(0, i) = lhs.template block<3,1>(0, i).cross(rhs);
    }
  };
  template <int ILhsCols>
  struct CrossProductCalculator<ILhsCols, 1> {
    template <typename TLhs, typename TRhs, typename TResult>
    inline static void calcCrossInto(const TLhs & lhs, const TRhs & rhs, TResult & result) {
      const int resultCols = result.cols();
      for(int i = 0; i < resultCols; i++) result.template block<3,1>(0, i) = lhs.template block<3,1>(0, i).cross(rhs);
    }
  };
  template <int IRhsCols>
  struct CrossProductCalculator<1, IRhsCols> {
    template <typename TLhs, typename TRhs, typename TResult>
    inline static void calcCrossInto(const TLhs & lhs, const TRhs & rhs, TResult & result) {
      const int resultCols = result.cols();
      for(int i = 0; i < resultCols; i++) result.template block<3,1>(0, i) = lhs.cross(rhs.template block<3, 1>(0, i));
    }
  };
  template <>
  struct CrossProductCalculator<1, 1> {
    template <typename TLhs, typename TRhs, typename TResult>
    inline static void calcCrossInto(const TLhs & lhs, const TRhs & rhs, TResult & result) {
      result = lhs.cross(rhs);
    }
  };
}

_TEMPLATE
template <int IColsOther, typename TOtherNode, typename std::enable_if<IRows == 3 && (ICols == 1 || IColsOther == 1), int>::type>
GenericMatrixExpression<3, (ICols == 1 ? IColsOther : ICols), TScalar> _CLASS::cross(const GenericMatrixExpression<3, IColsOther, TScalar, TOtherNode> & other) const {
  enum {RESULT_COLS = (ICols == 1 ? IColsOther : ICols)};
  typedef GenericMatrixExpression<3, RESULT_COLS, TScalar> result_t;
  typedef GenericMatrixExpression<3, IColsOther, TScalar, TOtherNode> other_t;
  typedef internal::CrossProductCalculator<ICols, IColsOther> calculator_t;

  class ResultNode : public result_t::template BinaryOperationResult<ResultNode, self_t, other_t> {
  public:
    typedef typename result_t::template BinaryOperationResult<ResultNode, self_t, other_t> base_t;

    virtual ~ResultNode() {}

    virtual void evaluateImplementation() const {
      calculator_t::calcCrossInto(this->getLhsNode().evaluate(), this->getRhsNode().evaluate(), this->_currentValue);
    }

    inline typename base_t::apply_diff_return_t applyLhsDiff(const typename base_t::lhs_t::tangent_vector_t & tangent_vector) const {
      auto rhs = this->getRhsNode().evaluate();
      typename base_t::apply_diff_return_t result(3, std::max(tangent_vector.cols(), rhs.cols()));
      calculator_t::calcCrossInto(tangent_vector, rhs, result);
      return result;
    }
    inline typename base_t::apply_diff_return_t applyRhsDiff(const typename base_t::rhs_t::tangent_vector_t & tangent_vector) const {
      auto lhs = this->getLhsNode().evaluate();
      typename base_t::apply_diff_return_t result(3, std::max(lhs.cols(), tangent_vector.cols()));
      calculator_t::calcCrossInto(lhs, tangent_vector, result);
      return result;
    }
  };

  return ResultNode::create(*this, other);
}
#undef _TEMPLATE
#undef _CLASS

}  // namespace backend
}  // namespace aslam
