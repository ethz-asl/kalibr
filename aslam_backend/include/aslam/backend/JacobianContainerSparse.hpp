#ifndef ASLAM_JACOBIAN_CONTAINER_SPARSE_HPP
#define ASLAM_JACOBIAN_CONTAINER_SPARSE_HPP

#include <sparse_block_matrix/sparse_block_matrix.h>
#include <aslam/Exceptions.hpp>
#include <map>
#include <set>
#include "DesignVariable.hpp"
#include "JacobianContainer.hpp"
#include "backend.hpp"
#include "util/CommonDefinitions.hpp"

namespace aslam {
  namespace backend {

    template<int Rows = Eigen::Dynamic>
    class JacobianContainerSparse : public JacobianContainer {
    public:
      SM_DEFINE_EXCEPTION(Exception, aslam::Exception);
      static constexpr const int RowsAtCompileTime = Rows;

      /// \brief The map type for storing Jacobians. Sorting the list by block index
      ///        simplifies computing the upper-diagonal of the Hessian matrix.
      typedef std::map<DesignVariable*, Eigen::MatrixXd, DesignVariable::BlockIndexOrdering> map_t;
      typedef DesignVariable::set_t set_t;

      JacobianContainerSparse(int rows, const std::size_t maxNumMatrices = 100)
          : aslam::backend::JacobianContainer(rows, maxNumMatrices)
      {
        SM_ASSERT_TRUE(Exception, Rows == Eigen::Dynamic || rows == Rows, "");
      }
      virtual ~JacobianContainerSparse();

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
      template<typename DERIVED = Eigen::MatrixXd>
      void add(const JacobianContainerSparse& rhs, const Eigen::MatrixBase<DERIVED>* applyChainRule = nullptr);

      /// \brief Add the rhs container to this one.
      inline void addTo(JacobianContainer& jc);

      /// \brief Add the rhs container to this one. Alternative approach suitable for large left-hand sides
      template<typename DERIVED = Eigen::MatrixXd>
      void addLargeLhs(const JacobianContainerSparse& rhs, const Eigen::MatrixBase<DERIVED>* applyChainRule = nullptr);

      /// \brief Add a jacobian to the list. If the design variable is not active, discard the value.
      virtual void add(DesignVariable* designVariable, const Eigen::Ref<const Eigen::MatrixXd>& Jacobian) override;
      /// \brief Add a jacobian to the list with identity chain rule. If the design variable is not active, discard the value.
      virtual void add(DesignVariable* designVariable) override;

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


      /// Check whether the entries corresponding to design variable \p dv are finite
      virtual bool isFinite(const DesignVariable& dv) const override;

      /// Get the Jacobian associated with a particular design variable \p dv
      const Eigen::MatrixXd& Jacobian(const DesignVariable* dv) const;

      /// \brief Apply the chain rule to the set of Jacobians.
      /// This may change the number of rows of this set of Jacobians
      /// by multiplying through by df_dx on the left.
      void applyChainRule(const Eigen::MatrixXd& df_dx);

      /// \brief Clear the contents of this container
      void clear();

      /// \brief Set all entries to zero
      inline void setZero();

      /// \brief Clean and set the number of rows
      void reset(int rows);
      
      /// \brief Gets a sparse matrix with the Jacobians. The matrix is, in fact, dense
      ///        and the Jacobian ordering matches the sort order.
      SparseBlockMatrix asSparseMatrix() const;

      /// \brief Get a sparse matrix with the Jacobians. This uses the column block indices
      ///        to build the sparse, full width jacobian matrix
      SparseBlockMatrix asSparseMatrix(const std::vector<int>& colBlockIndices) const;

      /// \brief Gets a dense matrix with the Jacobians. The Jacobian ordering matches the sort order.
      virtual Eigen::MatrixXd asDenseMatrix() const override;

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


      template <typename MATRIX>
      void addJacobian(DesignVariable * dv, const MATRIX & jacobian);

      friend class internal::JacobianContainerImplHelper;

      /// \brief The list of design variables.
      map_t _jacobianMap;
    };

  } // namespace backend
} // namespace aslam


#include "implementation/JacobianContainerSparseImpl.hpp"


#endif /* ASLAM_JACOBIAN_CONTAINER_SPARSE_HPP */
