#ifndef ASLAM_DESIGN_VARIABLE_HPP
#define ASLAM_DESIGN_VARIABLE_HPP

#include <sm/Id.hpp>
#include <unordered_set>
#include <set>

#include <Eigen/Core>

#include <aslam/Exceptions.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {
  namespace backend {

    class JacobianContainer;

    class DesignVariable {
    public:
      /**
       * \struct BlockIndexOrdering
       *
       * A comparator to allow design variables to be ordered by block index.
       */
      struct BlockIndexOrdering {
        inline bool operator()(const DesignVariable* lhs, const DesignVariable* rhs) const {
          SM_ASSERT_TRUE_DBG(aslam::Exception, lhs != NULL, "Null value!");
          SM_ASSERT_TRUE_DBG(aslam::Exception, rhs != NULL, "Null value!");
          return lhs->blockIndex() < rhs->blockIndex();
        }
      };

      /**
       * \typedef unordered_set_t
       * \brief A fast unordered set of design variables.
       */
      typedef std::unordered_set< DesignVariable* > set_t;

        typedef boost::shared_ptr< DesignVariable > Ptr;

      /**
       * \typedef blockordered_set_t
       * \brief a set of design variables ordered by block index.
       */
      //      typedef std::unordered_set//set<DesignVariable *, BlockIndexOrdering> blockordered_set_t;

      DesignVariable();

      virtual ~DesignVariable();

      /// \brief what is the number of dimensions of the minimal perturbation.
      virtual int minimalDimensions() const;

      /// \brief update the design variable.
      void update(const double* update, int size);

      /// \brief Revert the last state update
      void revertUpdate();

      /// \brief is this design variable active in the optimization.
      bool isActive() const;

      /// \brief set the active state of this design variable.
      void setActive(bool active);

      /// \brief should this variable be marginalized in the Schur-complement step?
      bool isMarginalized() const;

      /// \brief should this variable be marginalized in the Schur-complement step?
      void setMarginalized(bool marginalized);

      /// \brief get the block index used in the optimization routine. -1 if not being optimized.
      int blockIndex() const;

      /// \brief set the block index used in the optimization routine.
      void setBlockIndex(int blockIndex);

      /// \brief set the scaling of this design variable used in the optimization.
      void setScaling(double scaling);

      /// \brief get the scaling of this design variable used in the optimization.
      double scaling() const;

      /// \brief The column base of this block in the Jacobian matrix
      int columnBase() const;

      /// \brief Set the column base of this block in the Jacobian matrix
      void setColumnBase(int columnBase);

      /// Returns the content of the design variable
      void getParameters(Eigen::MatrixXd& value) const;

      /// Sets the content of the design variable
      void setParameters(const Eigen::MatrixXd& value);

      /// \brief Computes the minimal distance in tangent space between the current value of the DV and xHat
      void minimalDifference(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

      /// \brief Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
      void minimalDifferenceAndJacobian(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;


    protected:
      /// \brief what is the number of dimensions of the perturbation variable.
      virtual int minimalDimensionsImplementation() const = 0;

      /// \brief Update the design variable.
      virtual void updateImplementation(const double* dp, int size) = 0;

      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation() = 0;

      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value)
        const = 0;

      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value)
        = 0;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat
      virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
      virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;



    private:
      /// \brief The block index used in the optimization routine.
      int _blockIndex;

      /// \brief The column base of this block in the Jacobian matrix
      int _columnBase;

      /// \brief Should this variable be marginalized in the schur-complement step?
      bool _isMarginalized;

      /// \brief is the design variable active in the optimization?
      bool _isActive;

      /// \brief The scaling of this design variable within the optimization.
      double _scaling;

    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_DESIGN_VARIABLE_HPP */
