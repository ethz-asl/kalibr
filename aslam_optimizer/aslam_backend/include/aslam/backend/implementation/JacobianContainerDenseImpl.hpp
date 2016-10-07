#include <type_traits>

#include "JacobianContainerImpl.hpp"
#include <aslam/backend/JacobianContainerDense.hpp>
#include <sm/assert_macros.hpp>

#define JACOBIAN_CONTAINER_DENSE_TEMPLATE template <typename Container, int Rows>
#define JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE JacobianContainerDense<Container, Rows>

namespace aslam {
namespace backend {

JACOBIAN_CONTAINER_DENSE_TEMPLATE
template<typename dummy>
JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::JacobianContainerDense(int rows, int cols, dummy)
    : JacobianContainer(rows), _jacobian(rows, cols)
{
  SM_ASSERT_TRUE(Exception, Rows == Eigen::Dynamic || _jacobian.rows() == Rows, "");
  clear();
}

JACOBIAN_CONTAINER_DENSE_TEMPLATE
bool JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::isFinite(const DesignVariable& dv) const
{
  SM_ASSERT_GE_LE(Exception, dv.columnBase(), 0, _jacobian.cols() - dv.minimalDimensions(), "");
  return _jacobian.block(0, dv.columnBase(), _jacobian.rows(), dv.minimalDimensions()).allFinite();
}

JACOBIAN_CONTAINER_DENSE_TEMPLATE
Eigen::MatrixXd JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::Jacobian(const DesignVariable* dv) const
{
  SM_ASSERT_GE_LE(Exception, dv->columnBase(), 0, _jacobian.cols() - dv->minimalDimensions(), "");
  return _jacobian.block(0, dv->columnBase(), _jacobian.rows(), dv->minimalDimensions());
}

/// \brief Clear the contents of this container
JACOBIAN_CONTAINER_DENSE_TEMPLATE
void JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::clear()
{
  _jacobian.setZero();
}

JACOBIAN_CONTAINER_DENSE_TEMPLATE
template<typename MATRIX>
EIGEN_ALWAYS_INLINE void JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::addJacobian(DesignVariable* dv, const MATRIX& Jacobian)
{
  SM_ASSERT_EQ_DBG(Exception, _jacobian.rows(), Jacobian.rows(), ""); // This is very unlikely since the chain rule size is checked in JacobianContainer::apply()
  SM_ASSERT_GE_LE_DBG(Exception, dv->columnBase(), 0, _jacobian.cols() - Jacobian.cols(), "Check that column base of design variable is set correctly");
  _jacobian.block( 0, dv->columnBase(), Jacobian.rows(), Jacobian.cols() ).noalias() += Jacobian;
}

JACOBIAN_CONTAINER_DENSE_TEMPLATE
template<typename DERIVED>
void JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::add(const JacobianContainerDense& rhs, const Eigen::MatrixBase<DERIVED>* applyChainRule /*= nullptr*/)
{
  SM_THROW(NotImplementedException, __PRETTY_FUNCTION__ << " not implemented");
}

JACOBIAN_CONTAINER_DENSE_TEMPLATE
void JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::add(DesignVariable* dv, const Eigen::Ref<const Eigen::MatrixXd>& Jacobian)
{
  internal::JacobianContainerImplHelper::addImpl(*this, dv, Jacobian);
}

JACOBIAN_CONTAINER_DENSE_TEMPLATE
void JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE::add(DesignVariable* designVariable)
{
  internal::JacobianContainerImplHelper::addImpl(*this, designVariable);
}


// Explicit template instantiation
extern template class JacobianContainerDense<Eigen::MatrixXd, Eigen::Dynamic>;
extern template class JacobianContainerDense<Eigen::MatrixXd, 1>;
extern template class JacobianContainerDense<Eigen::MatrixXd, 2>;
extern template class JacobianContainerDense<Eigen::MatrixXd&, Eigen::Dynamic>;
extern template class JacobianContainerDense<Eigen::MatrixXd&, 1>;
extern template class JacobianContainerDense<Eigen::MatrixXd&, 2>;

#undef JACOBIAN_CONTAINER_DENSE_TEMPLATE
#undef JACOBIAN_CONTAINER_DENSE_CLASS_TEMPLATE

} // namespace backend
} // namespace aslam
