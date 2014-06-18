#ifndef ASLAM_JACOBIAN_CONTAINER_HPP
#define ASLAM_JACOBIAN_CONTAINER_HPP

#include <sparse_block_matrix/sparse_block_matrix.h>
#include <aslam/Exceptions.hpp>
#include <map>
#include <set>
#include "DesignVariable.hpp"
#include "backend.hpp"

namespace aslam {
  namespace backend {

    class JacobianContainer {
    public:
      SM_DEFINE_EXCEPTION(Exception, aslam::Exception);

      /// \brief The map type for storing Jacobians. Sorting the list by block index
      ///        simplifies computing the upper-diagonal of the Hessian matrix.
      typedef std::map<DesignVariable*, Eigen::MatrixXd, DesignVariable::BlockIndexOrdering> map_t;
      typedef DesignVariable::set_t set_t;

      JacobianContainer(int rows);
      virtual ~JacobianContainer();

      /**
       * \brief Evaluate the Hessian and the RHS of Gauss-Newton given the error and inverse covariance.
       *
       * Only the upper triangular portion of the sparse matrix will be filled in.
       * This is the full Hessian matrix of the whole optimization problem. The correct index
       * for each design variable can be found using dv.blockIndex
       *
       * @param e          The error used for evaluation of the Hessian
       * @param invR       The inverse uncertainty used for evaluation of the Hessian
       * @param outHessian After evaluation this Hessian matrix will be filled in with J^T inv(R) J
       * @param outRhs     After evaluation this vector will be filled in with J^T inv(R) e
       */
      void evaluateHessian(const Eigen::VectorXd& e, const Eigen::MatrixXd& invR, SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs) const;

      /// \brief Add the rhs container to this one.
      void add(const JacobianContainer& rhs);

      /// \brief Add a jacobian to the list. If the design variable is not active,
      /// discard the value.
      template<typename DERIVED>
      void add(DesignVariable* designVariable, const Eigen::MatrixBase<DERIVED>& Jacobian);


      /// \brief how many design variables does this jacobian container represent.
      size_t numDesignVariables() const;

      /// \brief Get design variable i.
      DesignVariable* designVariable(size_t i);

      /// \brief Get design variable i.
      const DesignVariable* designVariable(size_t i) const;

      map_t::const_iterator begin() const;
      map_t::const_iterator end() const;

      map_t::iterator begin();
      map_t::iterator end();


      /// Get the Jacobian associated with a particular design variable.
      const Eigen::MatrixXd& Jacobian(const DesignVariable* dv) const;

      /// \brief How many rows does this set of Jacobians have?
      int rows() const;

      /// \brief Apply the chain rule to the set of Jacobians.
      /// This may change the number of rows of this set of Jacobians
      /// by multiplying through by df_dx on the left.
      void applyChainRule(const Eigen::MatrixXd& df_dx);

      /// \brief Clear the contents of this container
      void clear();

      /// \brief Clean and set the number of rows
      void reset(int rows);
      
      /// \brief Gets a sparse matrix with the Jacobians. The matrix is, in fact, dense
      ///        and the Jacobian ordering matches the sort order.
      SparseBlockMatrix asSparseMatrix() const;

      /// \brief Get a sparse matrix with the Jacobians. This uses the column block indices
      ///        to build the sparse, full width jacobian matrix
      SparseBlockMatrix asSparseMatrix(const std::vector<int>& colBlockIndices) const;

      /// \brief Gets a dense matrix with the Jacobians. The Jacobian ordering matches the sort order.
      Eigen::MatrixXd asDenseMatrix() const;
      /// \brief Gets a dense matrix with the Jacobians. This uses the column block indices
      ///        to build the sparse, full width jacobian matrix
      Eigen::MatrixXd asDenseMatrix(const std::vector<int>& colBlockIndices) const;

      /// The number of columns in the compressed Jacobian. Warning: this is expensive.
      int cols() const;
    private:

      void buildCorrelatedHessianBlock(const Eigen::VectorXd& e,
                                       const Eigen::MatrixXd& J1, int j1_block,
                                       SparseBlockMatrix& outHessian,
                                       map_t::const_iterator it, map_t::const_iterator it_end) const;

      void buildHessianBlock(const Eigen::VectorXd& e,
                             SparseBlockMatrix& outHessian,
                             Eigen::VectorXd& outRhs,
                             map_t::const_iterator it, map_t::const_iterator it_end) const;


      /// \brief The number of rows for this set of Jacobians
      int _rows;

      /// \brief The list of design variables.
      map_t _jacobianMap;
    };


    template<typename DERIVED>
    void JacobianContainer::add(DesignVariable* dv, const Eigen::MatrixBase<DERIVED>& Jacobian)
    {
      SM_ASSERT_EQ(Exception, Jacobian.rows(), _rows, "The Jacobian must have the same number of rows as this container");
      SM_ASSERT_EQ(Exception, Jacobian.cols(), dv->minimalDimensions(), "The Jacobian must have the same number of cols as dv->minimalDimensions()");
      // If the designe variable isn't active. Don't bother adding it.
      if (! dv->isActive())
        return;
      SM_ASSERT_GE_DBG(Exception, dv->blockIndex(), 0, "The design variable is active but the block index is less than zero.");
      map_t::iterator it = _jacobianMap.find(dv);
      if (it == _jacobianMap.end()) {
        _jacobianMap.insert(_jacobianMap.end(), std::make_pair(dv, Eigen::MatrixXd(Jacobian.template cast<double>())));
      } else {
        SM_ASSERT_TRUE_DBG(Exception, it->first == dv, "Two design variables had the same block index but different pointer values");
        it->second += Jacobian.template cast<double>();
      }
    }




  } // namespace backend
} // namespace aslam


#endif /* ASLAM_JACOBIAN_CONTAINER_HPP */
