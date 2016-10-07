#ifndef ASLAM_JACOBIAN_CONTAINER_SPARSE_IMPL_HPP
#define ASLAM_JACOBIAN_CONTAINER_SPARSE_IMPL_HPP

#include <sm/assert_macros.hpp>

#include "JacobianContainerImpl.hpp"

#define JACOBIAN_CONTAINER_SPARSE_TEMPLATE template <int Rows>
#define JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE aslam::backend::JacobianContainerSparse<Rows>

namespace aslam {
  namespace backend {

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::~JacobianContainerSparse()
    {
    }

    /// \brief how many design variables does this jacobian container represent.
    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    size_t JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::numDesignVariables() const
    {
      return _jacobianMap.size();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::map_t::const_iterator JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::begin() const
    {
      return _jacobianMap.begin();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::map_t::const_iterator JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::end() const
    {
      return _jacobianMap.end();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::map_t::iterator JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::begin()
    {
      return _jacobianMap.begin();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::map_t::iterator JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::end()
    {
      return _jacobianMap.end();
    }


    /// \brief Apply the chain rule to the set of Jacobians.
    /// This may change the number of rows of this set of Jacobians
    /// by multiplying through by df_dx on the left.
    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::applyChainRule(const Eigen::MatrixXd& df_dx)
    {
      SM_ASSERT_EQ(Exception, df_dx.cols(), _rows, "Invalid matrix multiplication");
      map_t::iterator it = _jacobianMap.begin(),
                      it_end = _jacobianMap.end();
      for (; it != it_end; ++it)
        it->second = df_dx * it->second;
      _rows = df_dx.rows();
    }



    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::buildCorrelatedHessianBlock(const Eigen::VectorXd& e,
                                                        const Eigen::MatrixXd& J1, int j1_block,
                                                        SparseBlockMatrix& outHessian,
                                                        map_t::const_iterator it, map_t::const_iterator it_end) const
    {
      // Recursion base case.
      if (it == it_end)
        return;
      int j2_block = it->first->blockIndex();
      SM_ASSERT_LE_DBG(Exception, j1_block, j2_block, "The recursion should proceed in ordered blocks in order to only populate the upper diagonal of the Hessian. This violates the ordering.");
      const bool allocateIfMissing = true;
      Eigen::MatrixXd* J1t_invR_J2 = outHessian.block(j1_block, j2_block, allocateIfMissing);
      SM_ASSERT_TRUE_DBG(Exception, J1t_invR_J2 != NULL, "The Hessian block is NULL");
      SM_ASSERT_EQ_DBG(Exception, J1t_invR_J2->rows(), J1.cols(),
                       "The Hessian block has an unexpected number of rows. Block J1^T invR J2: (" <<
                       j1_block << ", " << j2_block << "). J1 is: " <<
                       J1.cols() << "x" << J1.rows() <<  ", J2 is: " <<
                       it->second.rows() << "x" << it->second.cols());
      SM_ASSERT_EQ_DBG(Exception, J1t_invR_J2->cols(), it->second.cols(),
                       "The Hessian block has an unexpected number of rows. Block J1^T invR J2: (" <<
                       j1_block << ", " << j2_block << "). J1 is: " <<
                       J1.cols() << "x" << J1.rows() <<  ", J2 is: " <<
                       it->second.rows() << "x" << it->second.cols());
      *J1t_invR_J2 += J1.transpose() * it->second;
      // Tail recursion
      buildCorrelatedHessianBlock(e, J1, j1_block, outHessian, ++it, it_end);
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::buildHessianBlock(const Eigen::VectorXd& e,
                                              SparseBlockMatrix& outHessian,
                                              Eigen::VectorXd& outRhs,
                                              map_t::const_iterator it, map_t::const_iterator it_end) const
    {
      // Recursion base case.
      if (it == it_end)
        return;
      const int j1_block = it->first->blockIndex();
      // \todo Make this a debug assert.
      SM_ASSERT_NE_DBG(Exception, j1_block, -1, "Negative blocks shouldn't make it in here");
      // templated version: outRhs.segment< J1::ColsAtCompileTime >.(j1_block);
      outRhs.segment(outHessian.rowBaseOfBlock(j1_block), it->second.cols()) -= it->second.transpose() * e;
      // Recurse in to the list to build correlated blocks
      buildCorrelatedHessianBlock(e, it->second, j1_block, outHessian, it, it_end);
      // Tail recursion to traverse the list
      buildHessianBlock(e, outHessian, outRhs, ++it, it_end);
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::evaluateHessian(const Eigen::VectorXd& e, const Eigen::MatrixXd& sqrtInvR, SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs) const
    {
      SM_ASSERT_EQ_DBG(Exception, e.size(), _rows, "The error and this Jacobian container should have the same size");
      SM_ASSERT_EQ_DBG(Exception, e.size(), sqrtInvR.rows(), "The error and the covariance matrix don't have compatible sizes");
      //SparseMatrixBlock * block = outHessian.block(int r, int c, allocIfMissing);
      // This sucks but I'll have to take a copy of the map to keep this function const.
      map_t mapCopy = _jacobianMap;
      // Scale each Jacobian.
      map_t::iterator itt = mapCopy.begin();
      for (; itt != mapCopy.end(); ++itt) {
        itt->second = (itt->first->scaling() * sqrtInvR.transpose() * itt->second).eval();
      }
      map_t::const_iterator it = mapCopy.begin();
      map_t::const_iterator it_end = mapCopy.end();
      // Start the recursion
      buildHessianBlock(sqrtInvR.transpose() * e, outHessian, outRhs, it, it_end);
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    bool JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::isFinite(const DesignVariable& dv) const
    {
      map_t::const_iterator it = _jacobianMap.find(const_cast<DesignVariable*>(&dv));
      SM_ASSERT_TRUE(Exception, it != _jacobianMap.end(), "The design variable does not exist in the container");
      return it->second.allFinite();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    const Eigen::MatrixXd& JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::Jacobian(const DesignVariable* dv) const
    {
      map_t::const_iterator it = _jacobianMap.find(const_cast<DesignVariable* >(dv));
      SM_ASSERT_TRUE(Exception, it != _jacobianMap.end(), "The design variable does not exist in the container");
      return it->second;
    }

    /// \brief Get design variable i.
    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    DesignVariable* JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::designVariable(size_t i)
    {
      SM_ASSERT_LT(Exception, i, numDesignVariables(), "Index out of range");
      map_t::iterator it = _jacobianMap.begin();
      for (size_t ii = 0; ii < i; ++ii)
        it++;
      return it->first;
    }

    /// \brief Get design variable i.
    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    const DesignVariable* JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::designVariable(size_t i) const
    {
      SM_ASSERT_LT(Exception, i, numDesignVariables(), "Index out of range");
      map_t::const_iterator it = _jacobianMap.begin();
      for (size_t ii = 0; ii < i; ++ii)
        it++;
      return it->first;
    }

  JACOBIAN_CONTAINER_SPARSE_TEMPLATE
  void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::reset(int rows) {
    clear();
    _rows = rows;
  }
  
    /// \brief Clear the contents of this container
    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::clear()
    {
      _jacobianMap.clear();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::setZero() {
      for (auto& dvJacPair : _jacobianMap)
        dvJacPair.second.setZero();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    Eigen::MatrixXd JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::asDenseMatrix() const
    {
      // \todo make efficient
      return asSparseMatrix().toDense();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    Eigen::MatrixXd JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::asDenseMatrix(const std::vector<int>& colBlockIndices) const
    {
      // \todo make efficient...however, these are only for debugging and unit testing.
      return asSparseMatrix(colBlockIndices).toDense();
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    SparseBlockMatrix JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::asSparseMatrix(const std::vector<int>& colBlockIndices) const
    {
      // \todo error checking.
      std::vector<int> rows(1);
      rows[0] = _rows;
      /// Step 2: fill the Jacobian
      SparseBlockMatrix J(rows, colBlockIndices, true);
      map_t::const_iterator it = _jacobianMap.begin();
      for (int col = 0; it != _jacobianMap.end(); ++it, col++) {
        const bool allocateBlock = true;
        SM_ASSERT_GE_LT_DBG(aslam::IndexOutOfBoundsException, (size_t)it->first->blockIndex(), 0, colBlockIndices.size(), "Block index is out of bounds");
        Eigen::MatrixXd& Ji = *J.block(0, it->first->blockIndex(), allocateBlock);
        Ji = it->second;
      }
      return J;
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    SparseBlockMatrix JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::asSparseMatrix() const
    {
      /// Step 1: Determine the block structure of the Jacobian.
      std::vector<int> rows(1);
      rows[0] = _rows;
      std::vector<int> cols(numDesignVariables());
      int sum = 0;
      map_t::const_iterator it = _jacobianMap.begin();
      for (int i = 0 ; it != _jacobianMap.end(); ++it, ++i) {
        sum += it->first->minimalDimensions();
        cols[i] = sum;
      }
      /// Step 2: fill the Jacobian
      SparseBlockMatrix J(rows, cols, true);
      it = _jacobianMap.begin();
      for (int col = 0; it != _jacobianMap.end(); ++it, col++) {
        const bool allocateBlock = true;
        Eigen::MatrixXd& Ji = *J.block(0, col, allocateBlock);
        Ji = it->second;
      }
      return J;
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    int JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::cols() const
    {
      int sum = 0;
      map_t::const_iterator it = _jacobianMap.begin();
      for (int i = 0 ; it != _jacobianMap.end(); ++it, ++i) {
        sum += it->first->minimalDimensions();
      }
      return sum;
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    template <typename MATRIX>
    EIGEN_ALWAYS_INLINE void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::addJacobian(DesignVariable* dv, const MATRIX& jacobian)
    {
      map_t::iterator it = _jacobianMap.find(dv);
      if (it == _jacobianMap.end()) {
          _jacobianMap.emplace(dv, jacobian);
      } else {
        SM_ASSERT_TRUE_DBG(Exception, it->first == dv, "Two design variables had the same block index but different pointer values");
          it->second.noalias() += jacobian;
      }
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::add(DesignVariable* dv, const Eigen::Ref<const Eigen::MatrixXd>& Jacobian)
    {
      internal::JacobianContainerImplHelper::addImpl(*this, dv, Jacobian);
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::add(DesignVariable* designVariable)
    {
      internal::JacobianContainerImplHelper::addImpl(*this, designVariable);
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    template<typename DERIVED>
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::add(const JacobianContainerSparse& rhs, const Eigen::MatrixBase<DERIVED>* applyChainRule /*= nullptr*/)
    {
      SM_ASSERT_EQ(Exception, _rows, rhs._rows, "The JacobianContainers cannot be added. They don't have the same number of rows.");
      if (applyChainRule != nullptr)
        SM_ASSERT_EQ(Exception, applyChainRule->cols(), rhs._rows, "Wrong dimension of chain rule matrix");
      // Merge the two maps.
      // They are sorted by block it->first->blockIndex() so we can be smart about this.
      map_t::iterator lt = _jacobianMap.begin();
      const map_t::iterator lt_end = _jacobianMap.end();
      map_t::const_iterator rt = rhs._jacobianMap.begin();
      map_t::const_iterator rt_end = rhs._jacobianMap.end();
      bool done = rt == rt_end;
      for (; lt != lt_end && !done; ++lt) {
        // The maps are sorted by block index. FFWD the rhs map
        // inserting elements until we find the next possible
        // equal element.
        while (!done && rt->first->blockIndex() < lt->first->blockIndex()) {
          _jacobianMap.insert(lt, std::make_pair(rt->first, applyChainRule == nullptr ? rt->second : (*applyChainRule)*rt->second));
          ++rt;
          done = (rt == rt_end);
        }
        // If these keys match the Jacobians add
        if (!done && rt->first->blockIndex() == lt->first->blockIndex()) {
          SM_ASSERT_TRUE_DBG(Exception, rt->first == lt->first, "Two design variables had the same block index but different pointer values");
          // add the Jacobians.
          lt->second += applyChainRule == nullptr ? rt->second : (*applyChainRule)*rt->second;
        }
      }
      // Now the lhs list is done...add the remaining elements of the rhs list.
      for (; rt != rt_end; rt++) {
        map_t::iterator it = _jacobianMap.insert(lt, std::make_pair(rt->first, applyChainRule == nullptr ? rt->second : (*applyChainRule)*rt->second));
      }
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    template<typename DERIVED>
    void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::addLargeLhs(const JacobianContainerSparse& rhs, const Eigen::MatrixBase<DERIVED>* applyChainRule /*= nullptr*/)
    {
      for (auto dvJacPair : rhs)
        add(dvJacPair.first, applyChainRule == nullptr ? dvJacPair.second : (*applyChainRule)*dvJacPair.second);
    }

    JACOBIAN_CONTAINER_SPARSE_TEMPLATE
    inline void JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE::addTo(JacobianContainer& jc)
    {
      for (auto& dvJacPair : _jacobianMap)
        jc.add(dvJacPair.first, dvJacPair.second);
    }

    // Explicit template instantiation
    extern template class JacobianContainerSparse<Eigen::Dynamic>;
    extern template class JacobianContainerSparse<1>;
    extern template class JacobianContainerSparse<2>;

  } // namespace backend
} // namespace aslam


#undef JACOBIAN_CONTAINER_SPARSE_TEMPLATE
#undef JACOBIAN_CONTAINER_SPARSE_CLASS_TEMPLATE


#endif /* ASLAM_JACOBIAN_CONTAINER_SPARSE_IMPL_HPP */

