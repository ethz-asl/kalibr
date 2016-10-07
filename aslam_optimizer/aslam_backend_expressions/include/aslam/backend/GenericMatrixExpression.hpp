#ifndef ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_HPP
#define ASLAM_BACKEND_GENERIC_MATRIX_EXPRESSION_HPP

#include <set>
#include <type_traits>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <aslam/backend/JacobianContainer.hpp>
#include "ScalarExpression.hpp"
#include "ScalarExpressionNode.hpp"
#include "GenericMatrixExpressionNode.hpp"
#include "OperationResultNodes.hpp"
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
  enum { Dimension = IRows*ICols };
  typedef GenericMatrixExpression self_t;
  typedef GenericMatrixExpressionNode<IRows, ICols, TScalar> default_node_t;
  typedef TNode node_t;
  typedef GenericMatrixExpression<IRows, ICols, TScalar, default_node_t> default_self_t;

  typedef TScalar scalar_t;
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

  /// \brief Initialize with a TScalar in case of 1 x 1 matrix (allows implizit cast from TScalar)
  template<typename dummy = int>
  GenericMatrixExpression(TScalar v, dummy = typename std::enable_if<(IRows == 1 && ICols == 1), int>::type(0));

  /// \brief Initialize from a ScalarExpression in case of 1 x 1 matrix
  template<typename dummy = int>
  GenericMatrixExpression(ScalarExpression v, dummy = typename std::enable_if<(IRows == 1 && ICols == 1), int>::type(0));

  /// \brief Support cast to ScalarExpression in case of 1 x 1 matrix
  template<typename dummy = int, typename = typename std::enable_if<(IRows == 1 && ICols == 1), dummy>::type>
  operator ScalarExpression const (){ return toScalarExpression(); }

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

  /// \brief return the squared norm expression (only supported for column vectors)
  ScalarExpression squaredNorm() const;

  template<int IColsOther, typename TOtherNode>
  GenericMatrixExpression<IRows, IColsOther, TScalar> operator*(const GenericMatrixExpression<ICols, IColsOther, TScalar, TOtherNode> & other) const;

  template<typename Derived>
  GenericMatrixExpression<IRows, Derived::ColsAtCompileTime, TScalar> operator*(const Eigen::MatrixBase<Derived> & otherMatrix) const {
    return *this * GenericMatrixExpression<ICols, Derived::ColsAtCompileTime, TScalar>(otherMatrix);
  }

  inline operator GenericMatrixExpression<IRows, ICols, TScalar>() const;

  default_self_t operator*(const ScalarExpression & scalarExpression) const;
  default_self_t operator*(TScalar scalar) const;
  default_self_t operator-() const;
  template<typename TOtherNode>
  default_self_t operator+(const GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> & other) const;
  template <typename Derived>
  default_self_t operator+(const Eigen::MatrixBase<Derived> & m) const { return *this + default_self_t(m); }
  template<typename TOtherNode>
  default_self_t operator-(const GenericMatrixExpression<IRows, ICols, TScalar, TOtherNode> & other) const;
  template <typename Derived>
  default_self_t operator-(const Eigen::MatrixBase<Derived> & m) const { return *this - default_self_t(m); }

  template<int IColsOther, typename TOtherNode, typename std::enable_if<IRows == 3 && (ICols == 1 || IColsOther == 1), int>::type = 0>
  GenericMatrixExpression<3, (ICols == 1 ? IColsOther : ICols), TScalar> cross(const GenericMatrixExpression<3, IColsOther, TScalar, TOtherNode> & other) const;

  /// \brief Evaluate the Jacobians
  void evaluateJacobians(JacobianContainer & outJacobians) const;
  void evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const;
  void evaluateJacobians(JacobianContainer & outJacobians, const differential_t & diff) const;

  void getDesignVariables(DesignVariable::set_t & designVariables) const;

  ScalarExpression toScalarExpression() const;
  template <int RowIndex, int ColIndex>
  ScalarExpression toScalarExpression() const;

  ScalarExpression toScalarExpression(int row, int col = 0) const;

  ScalarExpression operator () (int row, int col = 0) const {
    return toScalarExpression(row, col);
  }
  ScalarExpression operator [] (int row) const {
    return toScalarExpression(row);
  }

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
  class UnaryOperationResult : public UnaryOperationResultNode<TDerived, TOperand, self_t, typename node_t::differential_t::domain_t, scalar_t> {
  };

  template<typename TDerived, typename TLhs, typename TRhs>
  class BinaryOperationResult : public BinaryOperationResultNode<TDerived, TLhs, TRhs, self_t, typename node_t::differential_t::domain_t, scalar_t> {
  };
};

template<int ICols, int IColsOther, typename TScalar, typename TNode, typename Derived>
GenericMatrixExpression<Derived::RowsAtCompileTime, IColsOther, TScalar> operator*(const Eigen::MatrixBase<Derived> & matrix, GenericMatrixExpression<ICols, IColsOther, TScalar, TNode> otherMatrix) {
  return  GenericMatrixExpression<Derived::RowsAtCompileTime, ICols, TScalar>(matrix) * otherMatrix;
}

template<int IRows, int ICols, typename TScalar, typename TNode, typename Derived>
GenericMatrixExpression<IRows, ICols, TScalar> operator+(const Eigen::MatrixBase<Derived> & matrix, GenericMatrixExpression<IRows, ICols, TScalar, TNode> otherMatrix) {
  return  GenericMatrixExpression<IRows, ICols, TScalar>(matrix) + otherMatrix;
}

template<int IRows, int ICols, typename TScalar, typename TNode, typename Derived>
GenericMatrixExpression<IRows, ICols, TScalar> operator-(const Eigen::MatrixBase<Derived> & matrix, GenericMatrixExpression<IRows, ICols, TScalar, TNode> otherMatrix) {
  return  GenericMatrixExpression<IRows, ICols, TScalar>(matrix) - otherMatrix;
}

}  // namespace backend
}  // namespace aslam

#include "implementation/GenericMatrixExpression.hpp"

#endif /* ASLAM_BACKEND_MATRIX_EXPRESSION_HPP */
