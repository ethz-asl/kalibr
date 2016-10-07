#ifndef ASLAM_JACOBIAN_CONTAINER_HPP
#define ASLAM_JACOBIAN_CONTAINER_HPP

#include <sparse_block_matrix/sparse_block_matrix.h>
#include <aslam/Exceptions.hpp>
#include <map>
#include <set>
#include "DesignVariable.hpp"
#include "backend.hpp"
#include "MatrixStack.hpp"
#include "util/CommonDefinitions.hpp"

namespace aslam {
  namespace backend {
  namespace internal {
    class JacobianContainerImplHelper;
  }

    /**
     * \struct JacobianContainerChainRuleApplied
     * \brief Helper return value struct for MatrixStack::PopGuard
     */
    struct JacobianContainerChainRuleApplied {
      JacobianContainerChainRuleApplied(MatrixStack::PopGuard&& pg)
          : _guard(std::move(pg))
      {

      }
      /// \brief Cast operator
      inline operator JacobianContainer&();
     private:
      MatrixStack::PopGuard _guard;
    };

    /**
     * \class JacobianContainer
     * \brief Interface for all Jacobian containers
     */
    class JacobianContainer : private MatrixStack {

     public:
      friend class JacobianContainerChainRuleApplied;
      template<typename DIFFERENTIAL>
      friend JacobianContainerChainRuleApplied applyDifferentialToJacobianContainer(JacobianContainer&, const DIFFERENTIAL&, int);

     public:

      /// \brief Constructor for a container with \p rows rows.
      ///        The chain rule matrix stack will reserve memory for \p maxNumMatrices matrices with 9 elements each
      JacobianContainer(int rows, const std::size_t maxNumMatrices = 100) : MatrixStack(rows, maxNumMatrices, 9), _rows(rows) { }

      /// \brief Destructor
      virtual ~JacobianContainer() { }

      /// \brief Add a jacobian to the list. If the design variable is not active, discard the value.
      virtual void add(DesignVariable* designVariable, const Eigen::Ref<const Eigen::MatrixXd>& Jacobian) = 0;

      /// \brief Add a jacobian to the list with identity chain rule. If the design variable is not active, discard the value.
      virtual void add(DesignVariable* designVariable) = 0;

      /// \brief Gets a dense matrix with the Jacobians. The Jacobian ordering matches the sort order.
      virtual Eigen::MatrixXd asDenseMatrix() const = 0;

      /// Check whether the entries corresponding to design variable \p dv are finite
      virtual bool isFinite(const DesignVariable& dv) const = 0;

      /// \brief How many rows does this set of Jacobians have?
      int rows() const { return _rows; }

      /// \brief How many rows does this set of Jacobians currently expect for a newly added matrix?
      int expectedRows() const { return chainRuleEmpty() ? rows() : numTopCols(); }

      /// \brief Push a matrix \p mat to the top of the stack
      template <typename DERIVED>
      EIGEN_ALWAYS_INLINE JacobianContainerChainRuleApplied apply(const Eigen::MatrixBase<DERIVED>& mat)
      {
        if (this->chainRuleEmpty())
          SM_ASSERT_EQ(Exception, this->rows(), mat.rows(), "");
        return JacobianContainerChainRuleApplied(this->pushWithGuard(mat));
      }

      /// \brief Push a scalar \p scalar to the top of the stack
      EIGEN_ALWAYS_INLINE JacobianContainerChainRuleApplied apply(const double scalar)
      {
        return JacobianContainerChainRuleApplied(this->pushWithGuard(scalar));
      }

      /// \brief Is the stack empty?
      bool chainRuleEmpty() const {
        return this->empty();
      }

      /// \brief Const getter for the chain rule matrix
      template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
      EIGEN_ALWAYS_INLINE MatrixStack::ConstMap<Rows, Cols> chainRuleMatrix() const {
        return this->top<Rows, Cols>();
      }

      /// \brief Mutable getter for the chain rule matrix
      template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
      EIGEN_ALWAYS_INLINE MatrixStack::Map<Rows, Cols> chainRuleMatrix() {
        return this->top<Rows, Cols>();
      }

      /// \brief Const getter for the \p i-th chain rule matrix in the history (0 is top matrix, -1 is previous and so on)
      ///        \p i has to be smaller or equal to zero!
      template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
      EIGEN_ALWAYS_INLINE MatrixStack::ConstMap<Rows, Cols> chainRuleMatrix(int i) const {
        return this->matrix<Rows, Cols>(static_cast<int>(this->numMatrices()) - 1 + i);
      }

      /// \brief Mutable getter for the \p i-th chain rule matrix in the history (0 is top matrix, -1 is previous and so on)
      ///        \p i has to be smaller or equal to zero!
      template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
      EIGEN_ALWAYS_INLINE MatrixStack::Map<Rows, Cols> chainRuleMatrix(const int i) {
        return this->matrix<Rows, Cols>(static_cast<int>(this->numMatrices()) - 1 + i);
      }

    protected:
      /// \brief The number of rows for this set of Jacobians
      int _rows;

    };


    inline JacobianContainerChainRuleApplied::operator JacobianContainer&()
    {
      return static_cast<JacobianContainer&>(_guard.stack());
    }

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_JACOBIAN_CONTAINER_HPP */
