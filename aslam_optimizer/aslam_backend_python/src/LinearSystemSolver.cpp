#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>
#include <aslam/backend/Matrix.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/SparseCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/SparseQrLinearSystemSolver.hpp>
#include <aslam/backend/DenseQrLinearSystemSolver.hpp>


/// \brief solve the system storing the solution in outDx and returning true on success.
//virtual bool solveSystem(Eigen::VectorXd& outDx) = 0;

boost::python::tuple solveSystem(aslam::backend::LinearSystemSolver * lss)
{
    Eigen::VectorXd dx;
    bool success = lss->solveSystem(dx);
    return boost::python::make_tuple(success, dx);
}


void exportLinearSystemSolver()
{
    using namespace boost::python;
    using namespace aslam::backend;

    class_<LinearSystemSolver, boost::shared_ptr<LinearSystemSolver>, boost::noncopyable>("LinearSystemSolver", no_init)
        .def("solveSystem", &solveSystem)
        /// \brief Evaluate the error using nThreads.
        .def("evaluateError", &LinearSystemSolver::evaluateError )

        /// \brief initialized the matrix structure for the problem with these error terms and errors.
        .def("initMatrixStructure", &LinearSystemSolver::initMatrixStructure )

        /// \brief build the system of equations.
        .def("buildSystem", &LinearSystemSolver::buildSystem )

        /// \brief Set the diagonal matrix conditioner.
        ///        NOTE: The square of these values will be added to the diagonal of the Hessian matrix
        .def("setConditioner", &LinearSystemSolver::setConditioner )

        /// \brief Set a constant diagonal conditioner.
        ///        NOTE: The square of this value will be added to the diagonal of the Hessian matrix
        .def("setConstantConditioner", &LinearSystemSolver::setConstantConditioner )


        /// \brief return the right-hand side of the equation system.
        .def("rhs", &LinearSystemSolver::rhs, return_value_policy<copy_const_reference>())

        /// \brief return the Jacobian matrix if available. Null if not available.
        .def("Jacobian", &LinearSystemSolver::Jacobian, return_internal_reference<>() )

        /// \brief return the Hessian matrix if avaliable. Null if not available.
        .def("Hessian", &LinearSystemSolver::Hessian, return_internal_reference<>() )

        /// \brief return the full error vector
        .def("e", &LinearSystemSolver::e, return_value_policy<copy_const_reference>())

        /// \brief the number of rows in the Jacobian matrix
        .def("JRows", &LinearSystemSolver::JRows )

        /// \brief the number of columns in the Jacobian matrix
        .def("JCols", &LinearSystemSolver::JCols )
      
        // helper function for dog leg implementation / steepest descent solution
        .def("rhsJtJrhs", &LinearSystemSolver::rhsJtJrhs )

        ;

    SparseQRLinearSolverOptions& (SparseQrLinearSystemSolver::*getOptions)() = &SparseQrLinearSystemSolver::getOptions;
      /// Sets the options


    class_<SparseQRLinearSolverOptions>("SparseQrLinearSolverOptions", init<>())
        .def_readwrite("colNorm", &SparseQRLinearSolverOptions::colNorm)
        .def_readwrite("qrTol", &SparseQRLinearSolverOptions::qrTol)
        ;


    class_<DenseQrLinearSystemSolver, boost::shared_ptr<DenseQrLinearSystemSolver>, bases<LinearSystemSolver> >("DenseQrLinearSystemSolver", init<>());
    class_<BlockCholeskyLinearSystemSolver, boost::shared_ptr<BlockCholeskyLinearSystemSolver>, bases<LinearSystemSolver> >("BlockCholeskyLinearSystemSolver", init<>());
    class_<SparseCholeskyLinearSystemSolver, boost::shared_ptr<SparseCholeskyLinearSystemSolver>, bases<LinearSystemSolver> >("SparseCholeskyLinearSystemSolver", init<>());
    class_<SparseQrLinearSystemSolver, boost::shared_ptr<SparseQrLinearSystemSolver>, bases<LinearSystemSolver> >("SparseQrLinearSystemSolver", init<>())
        .def("getJacobianTranspose", &SparseQrLinearSystemSolver::getJacobianTranspose, return_internal_reference<>())
        .def("getRank", &SparseQrLinearSystemSolver::getRank)
        .def("getTol", &SparseQrLinearSystemSolver::getTol)
        .def("getPermutationVector", &SparseQrLinearSystemSolver::getPermutationVectorEigen)
        .def("getR", &SparseQrLinearSystemSolver::getR, return_internal_reference<>())
        .def("getMemoryUsage", &SparseQrLinearSystemSolver::getMemoryUsage)
        .def("analyzeSystem", &SparseQrLinearSystemSolver::analyzeSystem)
        .def("getOptions", getOptions, return_internal_reference<>())
        .def("setOptions", &SparseQrLinearSystemSolver::setOptions)
        ;

}
