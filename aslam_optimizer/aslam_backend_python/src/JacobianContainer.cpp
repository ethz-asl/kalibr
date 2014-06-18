#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
using namespace boost::python;
using namespace aslam::backend;


DesignVariable * (JacobianContainer::*jc_dvptr)(size_t ) = &JacobianContainer::designVariable;

/// \brief Add the rhs container to this one.
void (JacobianContainer::*jc_add1)(const JacobianContainer & ) = &JacobianContainer::add;
      
void jc_add2(JacobianContainer * jc, DesignVariable & dv, Eigen::MatrixXd Jacobian)
{
  jc->add(&dv, Jacobian);
}

SparseBlockMatrix (JacobianContainer::*jc_asm1)() const = &JacobianContainer::asSparseMatrix;

SparseBlockMatrix jc_asm2(JacobianContainer * jc, const Eigen::VectorXi & colBlockIndices)
{
  std::vector<int> cbi;
  cbi.insert(cbi.begin(), &colBlockIndices[0], &colBlockIndices[0] + colBlockIndices.size());
  return jc->asSparseMatrix(cbi);
}

Eigen::MatrixXd (JacobianContainer::*jc_adm1)() const = &JacobianContainer::asDenseMatrix;

Eigen::MatrixXd jc_adm2(JacobianContainer * jc, const Eigen::VectorXi & colBlockIndices)
{
  std::vector<int> cbi;
  cbi.insert(cbi.begin(), &colBlockIndices[0], &colBlockIndices[0] + colBlockIndices.size());
  return jc->asDenseMatrix(cbi);
}



void exportJacobianContainer()
{
  class_<JacobianContainer>("JacobianContainer", init<int>())
      
    //void evaluateHessian(const Eigen::VectorXd & e, const Eigen::MatrixXd & invR, SparseBlockMatrix & outHessian, Eigen::VectorXd & outRhs) const;
      
    .def("add", jc_add1)
    .def("add", jc_add2)

    /// \brief how many design variables does this jacobian container represent.
    .def("numDesignVariables", &JacobianContainer::numDesignVariables)
            
    /// \brief Get design variable i.
    .def("designVariable", jc_dvptr, return_internal_reference<>())
      
    /// Get the Jacobian associated with a particular design variable.
    .def("Jacobian", &JacobianContainer::Jacobian, return_value_policy<copy_const_reference>())
      
    /// \brief How many rows does this set of Jacobians have?
    .def("rows", &JacobianContainer::rows)

    /// \brief Apply the chain rule to the set of Jacobians.
    /// This may change the number of rows of this set of Jacobians
    /// by multiplying through by df_dx on the left.
    .def("applyChainRule", &JacobianContainer::applyChainRule)
            
    /// \brief Clear the contents of this container
    .def("clear", &JacobianContainer::clear)

      /// \brief Clear the contents of this container
    .def("reset", &JacobianContainer::reset)

      
    /// \brief Gets a sparse matrix with the Jacobians. The matrix is, in fact, dense
    ///        and the Jacobian ordering matches the sort order. 
    /// 
    ///        \todo add a function that allows us to get the full block row
    ///        Jacobian. This is tricky because you have to know the
    ///        full block structure of the Hessian
    .def("asSparseMatrix", jc_asm1)
    /// \brief Get a sparse matrix with the Jacobians. This uses the column block indices
    ///        to build the sparse, full width jacobian matrix
    .def("asSparseMatrix", jc_asm2)

    .def("asDenseMatrix", jc_adm1)
    .def("asDenseMatrix", jc_adm2)
    
    /// The number of columns in the compressed Jacobian. Warning: this is expensive.
    .def("cols", &JacobianContainer::cols)
    ;
    }
