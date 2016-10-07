#ifndef ASLAM_JACOBIAN_CONTAINER_IMPL_HELPER_HPP
#define ASLAM_JACOBIAN_CONTAINER_IMPL_HELPER_HPP

#include <type_traits>
#include <sm/assert_macros.hpp>

namespace aslam {
namespace backend {
namespace internal {

class JacobianContainerImplHelper {
  template <typename JC, typename M1, typename M2>
  EIGEN_ALWAYS_INLINE static void multiplyAndAdd(JC & jc, DesignVariable* dv, const M1& m1, const M2& m2, std::false_type /*r2IsIdentity*/)
  {
    SM_ASSERT_EQ_DBG(Exception, m1.cols(), m2.rows(), "Invalid matrix product of chain rule matrix and Jacobian");
    jc.addJacobian(dv, m1*m2);
  }
  /// \brief This method is called for identity m2
  template <typename JC, typename M1, typename M2>
  EIGEN_ALWAYS_INLINE static void multiplyAndAdd(JC & jc, DesignVariable* dv, const M1& m1, const M2& /*m2*/, std::true_type /*isIdentity*/)
  {
    jc.addJacobian(dv, m1);
  }

 public:
  template<bool IS_IDENTITY = false, typename JC, typename MATRIX>
  EIGEN_ALWAYS_INLINE static void addImpl(JC & jc, DesignVariable* dv, const MATRIX & jacobian)
  {
    SM_ASSERT_EQ_DBG(Exception, jacobian.cols(), dv->minimalDimensions(), "The Jacobian must have the same number of cols as dv->minimalDimensions()");

    // If the design variable isn't active, don't bother adding it.
    if (! dv->isActive())
      return;
    SM_ASSERT_GE_DBG(Exception, dv->blockIndex(), 0, "The design variable is active but the block index is less than zero.");

    if (jc.chainRuleEmpty())
    {
      SM_ASSERT_EQ_DBG(Exception, jc.rows(), jacobian.rows(), "The Jacobian must have the same number of rows as this container");
      jc.addJacobian(dv, jacobian.template cast<double>());
    }
    else
    {
      const auto CR = jc.template chainRuleMatrix<JC::RowsAtCompileTime>();
      multiplyAndAdd(jc, dv, CR, jacobian.template cast<double>(), std::integral_constant<bool, IS_IDENTITY>());
    }
  }

  template<typename JC>
  EIGEN_ALWAYS_INLINE static void addImpl(JC & jc, DesignVariable* dv){
    // Note: The dimensions are not correct if the chain rule is not empty.
    // Yet in this case we skip the multiplication anyways!
    addImpl<true>(jc, dv, Eigen::MatrixXd::Identity(jc.rows(), dv->minimalDimensions()));
  }
};

} // namespace jc_internal
} // namespace backend
} // namespace aslam

#endif /* ASLAM_JACOBIAN_CONTAINER_IMPL_HELPER_HPP */
