#ifndef ASLAM_JACOBIAN_CONTAINER_DENSE_HPP
#define ASLAM_JACOBIAN_CONTAINER_DENSE_HPP

#include <aslam/Exceptions.hpp>
#include <vector>
#include "DesignVariable.hpp"
#include "JacobianContainer.hpp"
#include "backend.hpp"
#include "util/CommonDefinitions.hpp"

namespace aslam {
  namespace backend {

    template <typename Container, int Rows = Eigen::Dynamic>
    class JacobianContainerDense : public JacobianContainer {
    public:
      SM_DEFINE_EXCEPTION(Exception, aslam::Exception);
      typedef Container container_t;
      static constexpr const int RowsAtCompileTime = Rows;

      /// \brief Constructs the Jacobian container using \p jacobian as underlying data storage
      JacobianContainerDense(Container jacobian) : JacobianContainer(jacobian.rows()), _jacobian(jacobian)
      {
        SM_ASSERT_TRUE(Exception, Rows == Eigen::Dynamic || jacobian.rows() == Rows, "");
      }

      /// \brief Constructor with specified sizes \p rows and \p cols. Only enabled for non-reference type containers
      template<typename dummy = int>
      JacobianContainerDense(int rows, int cols, dummy = typename std::enable_if<!std::is_reference<Container>::value, int>::type(0));

      /// \brief Destructor
      virtual ~JacobianContainerDense() { }

      /// \brief Add the rhs container to this one.
      template<typename DERIVED = Eigen::MatrixXd>
      void add(const JacobianContainerDense& rhs, const Eigen::MatrixBase<DERIVED>* applyChainRule = nullptr);

      /// \brief Add a jacobian to the list. If the design variable is not active,
      /// discard the value.
      virtual void add(DesignVariable* designVariable, const Eigen::Ref<const Eigen::MatrixXd>& Jacobian) override;
      virtual void add(DesignVariable* designVariable) override;

      /// Check whether the entries corresponding to design variable \p dv are finite
      virtual bool isFinite(const DesignVariable& dv) const override;

      /// Get the Jacobian associated with a particular design variable \p dv
      Eigen::MatrixXd Jacobian(const DesignVariable* dv) const;

      /// \brief Clear the contents of this container
      void clear();

      /// \brief Gets a dense matrix with the Jacobians. The Jacobian ordering matches the sort order.
      virtual Eigen::MatrixXd asDenseMatrix() const override { return _jacobian; }

    private:

      template <typename MATRIX>
      void addJacobian(DesignVariable * dv, const MATRIX & jacobian);

      friend class internal::JacobianContainerImplHelper;
    private:

      /// \brief The data
      Container _jacobian;
    };

  } // namespace backend
} // namespace aslam

#include "implementation/JacobianContainerDenseImpl.hpp"

#endif /* ASLAM_JACOBIAN_CONTAINER_DENSE_HPP */
