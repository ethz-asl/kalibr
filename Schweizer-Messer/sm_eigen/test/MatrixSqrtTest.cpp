// Bring in my package's API, which is what I'm testing

#include <Eigen/Core>
// Bring in gtest
#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>
#include <boost/cstdint.hpp>

#include <sm/eigen/matrix_sqrt.hpp>


TEST(EigenMatrixSqrtTest, testMatrixSqrt)
{
    
    for(int i = 0; i < 100; ++i)
    {
        Eigen::MatrixXd R = sm::eigen::randomCovariance<3>();
        Eigen::MatrixXd sqrtR;
        sm::eigen::computeMatrixSqrt(R,sqrtR);
        Eigen::MatrixXd reconstructedR = sqrtR * sqrtR.transpose();
        ASSERT_DOUBLE_MX_EQ(R, reconstructedR, 1e-9, "Reconstructing the matrix from the square root in iteration " << i);
    }


}
