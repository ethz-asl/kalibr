#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/eigen/matrix_sqrt.hpp>
#include <sm/assert_macros.hpp>


Eigen::MatrixXd matrixSqrt(const Eigen::MatrixXd & M)
{
    SM_ASSERT_EQ(std::runtime_error, M.rows(), M.cols(), "The matrix must be square");
    Eigen::MatrixXd sqrtM;

    /*Eigen::ComputationInfo result =*/ sm::eigen::computeMatrixSqrt(M, sqrtM);
    //SM_ASSERT_EQ(std::runtime_error, result, Eigen::Success, "The matrix square root was not successful")
    return sqrtM;
}

void exportEigen()
{
    boost::python::def("computeMatrixSqrt", &matrixSqrt);
}
