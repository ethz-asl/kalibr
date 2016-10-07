#include <sm/eigen/gtest.hpp>
#include <vector>
#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/JacobianContainerDense.hpp>
#include <aslam/backend/JacobianContainerPrescale.hpp>
#include <aslam/backend/util/utils.hpp>
#include <numeric> // std::partial_sum
#include "DummyDesignVariable.hpp"
#include <Eigen/Cholesky>
#include <sm/eigen/matrix_sqrt.hpp>

template <typename JC>
struct JacobianContainerTests : public ::testing::Test  {
  virtual ~JacobianContainerTests() { }
  boost::shared_ptr<JC> getJacobianContainer(int rows, int cols) {
    return boost::shared_ptr<JC>(new JC(rows, cols));
  }
};

template <>
boost::shared_ptr< aslam::backend::JacobianContainerSparse<Eigen::Dynamic> >
JacobianContainerTests<aslam::backend::JacobianContainerSparse<Eigen::Dynamic> >::getJacobianContainer(int rows, int /*cols*/) {
  return boost::shared_ptr< aslam::backend::JacobianContainerSparse<Eigen::Dynamic> >(new aslam::backend::JacobianContainerSparse<Eigen::Dynamic>(rows));
}

typedef ::testing::Types<
    aslam::backend::JacobianContainerSparse<Eigen::Dynamic>,
    aslam::backend::JacobianContainerDense<Eigen::MatrixXd, Eigen::Dynamic>
> JacobianContainerTypes;

TYPED_TEST_CASE(JacobianContainerTests, JacobianContainerTypes);

template <int Dim>
std::vector< DummyDesignVariable<Dim> > createDesignVariables(const std::size_t num, const bool setActive = false)
{
  std::vector< DummyDesignVariable<Dim> > dvs(num);
  std::size_t blockIndex = 0, columnBase = 0;
  for (auto& dv : dvs) {
    dv.setBlockIndex(blockIndex);
    dv.setColumnBase(columnBase);
    dv.setActive(setActive);
    blockIndex += 1;
    columnBase += dv.minimalDimensions();
  }
  return dvs;
}

TEST(JacobianContainerTests, testAddSpecificForJacobianSparse)
{
  using namespace aslam::backend;

  // Test template argument check upon construction
  try {
    JacobianContainerSparse<1> jc(2, 3);
    FAIL() << "Construction with invalid number of rows did not fail";
  } catch (...) { }

  // Create a container with three rows.
  const int rows = 3;
  const int dim = 2;
  JacobianContainerSparse<rows> jc(rows);

  std::vector< DummyDesignVariable<2> > dvs = createDesignVariables<dim>(2, false);
  const auto J = Eigen::Matrix<double, rows, dim>::Random().eval();
  ASSERT_THROW(jc.Jacobian(&(dvs[0])), JacobianContainerSparse<rows>::Exception);
  ASSERT_EQ(jc.numDesignVariables(), 0u);
  // The container should skip inactive design variables.
  jc.add(&(dvs[0]), J);
  ASSERT_EQ(0u, jc.numDesignVariables());
  ASSERT_THROW(jc.Jacobian(&(dvs[0])), JacobianContainerSparse<rows>::Exception);
  dvs[0].setActive(true);
  jc.add(&(dvs[0]), J);
  ASSERT_EQ(1u, jc.numDesignVariables());

  ASSERT_THROW(jc.Jacobian(&(dvs[1])), JacobianContainerSparse<rows>::Exception);
  // The container should skip inactive design variables.
  jc.add(&(dvs[1]), J);
  ASSERT_EQ(1u, jc.numDesignVariables());
  dvs[1].setActive(true);
  jc.add(&(dvs[1]), J);
  ASSERT_EQ(2u, jc.numDesignVariables());
}

TEST(JacobianContainerTests, testAddSpecificForJacobianDense)
{
  try
  {
    using namespace aslam::backend;

    // Test template argument check upon construction
    try {
      JacobianContainerDense<Eigen::MatrixXd, 1> jc(2, 3);
      FAIL() << "Construction with invalid number of rows did not fail";
    } catch (...) { }
    try {
      auto J0 = Eigen::Matrix<double, 2, 3>::Random().eval();
      JacobianContainerDense<Eigen::Matrix<double, 2, 3>&, 1> jc(J0);
      FAIL() << "Construction with invalid number of rows did not fail";
    } catch (...) { }

    const int dimensionDv = 2;
    const int dimensionError = 3;

    auto dvs = createDesignVariables<dimensionDv>(2, false);
    std::size_t numDvParameters = dvs.back().columnBase() + dvs.back().minimalDimensions();

    // Test construction
    JacobianContainerDense<Eigen::MatrixXd> jc(dimensionError, numDvParameters);
    sm::eigen::assertEqual(jc.asDenseMatrix(), Eigen::MatrixXd::Zero(dimensionError, numDvParameters), SM_SOURCE_FILE_POS);

    const auto J0 = Eigen::Matrix<double, dimensionError, dimensionDv>::Random().eval();

    // The container should skip inactive design variables and return the zero matrix Jacobian in that case
    jc.add(&dvs[0], J0);
    sm::eigen::assertEqual(Eigen::MatrixXd::Zero(dimensionError, dimensionDv), jc.Jacobian(&(dvs[0])), SM_SOURCE_FILE_POS);
    sm::eigen::assertEqual(Eigen::MatrixXd::Zero(dimensionError, numDvParameters), jc.asDenseMatrix(), SM_SOURCE_FILE_POS);

    // One design variable active, other inactive
    dvs[0].setActive(true);
    jc.add(&dvs[0], J0);
    jc.add(&dvs[1], J0);
    sm::eigen::assertEqual(J0, jc.Jacobian(&(dvs[0])), SM_SOURCE_FILE_POS);
    sm::eigen::assertEqual(Eigen::MatrixXd::Zero(dimensionError, dimensionDv), jc.Jacobian(&(dvs[1])), SM_SOURCE_FILE_POS);
    auto expected = Eigen::Matrix<double, dimensionError, 2*dimensionDv>();
    expected.leftCols(dimensionDv) = J0;
    expected.rightCols(dimensionDv) = Eigen::MatrixXd::Zero(dimensionError, dimensionDv);
    sm::eigen::assertEqual(expected, jc.asDenseMatrix(), SM_SOURCE_FILE_POS);

    // Clearing the container should yield zero matrix
    jc.clear();
    sm::eigen::assertEqual(Eigen::MatrixXd::Zero(dimensionError, numDvParameters), jc.asDenseMatrix(), SM_SOURCE_FILE_POS);
  }
  catch (const std::exception& e)
  {
    FAIL() << "Exception: " << e.what();
  }
}

TYPED_TEST(JacobianContainerTests, testAddJacobian)
{
  try
  {
    using namespace aslam::backend;
    const int dimensionDv = 2;
    const int dimensionError = 3;

    auto dvs = createDesignVariables<dimensionDv>(2, true);
    std::size_t numDvParameters = dvs.back().columnBase() + dvs.back().minimalDimensions();
    boost::shared_ptr<TypeParam> jcPtr = this->getJacobianContainer(dimensionError, numDvParameters);
    TypeParam& jc = *jcPtr;

    const auto J0 = Eigen::Matrix<double, dimensionError, dimensionDv>::Random().eval();
    const auto J1 = Eigen::Matrix<double, dimensionError, dimensionDv>::Random().eval();

    // The container should add identity if add() is called without 2nd argument
    jc.add(&dvs[0]);
    sm::eigen::assertEqual(Eigen::MatrixXd::Identity(dimensionError, dimensionDv), jc.Jacobian(&dvs[0]), SM_SOURCE_FILE_POS);

    // Reset to zero
    jc.clear();

    // The container should add J0 to dv0
    jc.add(&dvs[0], J0);
    sm::eigen::assertEqual(J0, jc.Jacobian(&dvs[0]), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    // Adding J0 to dv0 again should yield J0 + J0.
    jc.add(&dvs[0], J0);
    sm::eigen::assertEqual(J0 + J0, jc.Jacobian(&dvs[0]), SM_SOURCE_FILE_POS, "Recover the Jacobian");


    // Add J1 to dv1
    jc.add(&dvs[1], J1);
    Eigen::MatrixXd expected = J1;
    sm::eigen::assertEqual(expected, jc.Jacobian(&dvs[1]), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    // Adding J1 to dv1 again should yield J1 + J1.
    jc.add(&dvs[1], J1);
    expected += J1;
    sm::eigen::assertEqual(expected, jc.Jacobian(&dvs[1]), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    // Test apply() method.
    // We add M*J1 to the existing content J1 + J1
    const auto M = Eigen::Matrix<double, dimensionError, dimensionError>::Random().eval();
    static_cast<JacobianContainer&>(jc.apply(M)).add(&dvs[1], J1);
    expected += M*J1;
    sm::eigen::assertEqual(expected, jc.Jacobian(&dvs[1]), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    // Test application of multiple chain rule matrices
    static_cast<JacobianContainer&>(static_cast<JacobianContainer&>(jc.apply(M)).apply(M)).add(&dvs[1], J1);
    expected += M*M*J1;
    sm::eigen::assertEqual(expected, jc.Jacobian(&dvs[1]), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    // Test that chain rule matrix is indeed popped from the stack and does not influence further add() calls
    jc.add(&dvs[1], J1);
    expected += J1;
    sm::eigen::assertEqual(expected, jc.Jacobian(&dvs[1]), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    // Test apply() method with identity.
    // We apply N as chain rule, this should add N to the existing content J1 + J1 + M*J1
    const auto N = Eigen::Matrix<double, dimensionError, dimensionDv>::Random().eval();
    static_cast<JacobianContainer&>(jc.apply(N)).add(&dvs[1]);
    expected += N;
    sm::eigen::assertEqual(expected, jc.Jacobian(&dvs[1]), SM_SOURCE_FILE_POS, "Recover the Jacobian");
  }
  catch (const std::exception& e)
  {
    FAIL() << "Exception: " << e.what();
  }
}

TEST(JacobianContainerTests, testAddJacobianPrescaled)
{
  try
  {
    using namespace aslam::backend;
    const int dimensionDv = 2;
    const int dimensionError = 3;
    const double scale = 2.0;
    auto dv = createDesignVariables<dimensionDv>(1, true)[0];

    const auto J0 = Eigen::Matrix<double, dimensionError, dimensionDv>::Random().eval();

    JacobianContainerSparse<dimensionError> jcs(dimensionError);
    JacobianContainerPrescaled jc(jcs, scale);

    jc.add(&dv, J0);
    Eigen::MatrixXd expected = scale*J0;
    sm::eigen::assertEqual(expected, jcs.Jacobian(&dv), SM_SOURCE_FILE_POS, "Recover the Jacobian");

    jc.add(&dv);
    expected += scale*Eigen::MatrixXd::Identity(dimensionError, dimensionDv);
    sm::eigen::assertEqual(expected, jcs.Jacobian(&dv), SM_SOURCE_FILE_POS, "Recover the Jacobian");

  }
  catch (const std::exception& e)
  {
    FAIL() << "Exception: " << e.what();
  }
}

TEST(JacobianContainerTests, testOrdering)
{
  using namespace aslam::backend;
  Eigen::Matrix2d J;
  JacobianContainerSparse<> jc(2);
  DummyDesignVariable<2> dv4;
  dv4.setBlockIndex(4);
  dv4.setActive(true);
  jc.add(&dv4, J);
  DummyDesignVariable<2> dv2;
  dv2.setBlockIndex(2);
  dv2.setActive(true);
  jc.add(&dv2, J);
  DummyDesignVariable<2> dv1;
  dv1.setBlockIndex(1);
  dv1.setActive(true);
  jc.add(&dv1, J);
  DummyDesignVariable<2> dv3;
  dv3.setBlockIndex(3);
  dv3.setActive(true);
  jc.add(&dv3, J);
  ASSERT_EQ(jc.numDesignVariables(), 4u);
  // Now check if the ordering is correct.
  // Jacobians should stored in ascending order
  // by block index
  JacobianContainerSparse<>::map_t::const_iterator itk = jc.begin(),
                                           itkm1 = jc.begin(),
                                           it_end = jc.end();
  itk++;
  for (; itk != it_end; ++itk, ++itkm1)
    ASSERT_LT((itkm1)->first->blockIndex(), itk->first->blockIndex());
}

TEST(JacobianContainerTests, testChainRule)
{
  try {
    using namespace aslam::backend;
    JacobianContainerSparse<> jc(2);
    DummyDesignVariable<1> dv1;
    dv1.setBlockIndex(1);
    dv1.setActive(true);
    Eigen::Matrix<double, 2, 1> J1;
    J1.setRandom();
    jc.add(&dv1, J1);
    DummyDesignVariable<2> dv2;
    dv2.setBlockIndex(2);
    dv2.setActive(true);
    Eigen::Matrix<double, 2, 2> J2;
    J2.setRandom();
    jc.add(&dv2, J2);
    DummyDesignVariable<3> dv3;
    dv3.setBlockIndex(3);
    dv3.setActive(true);
    Eigen::Matrix<double, 2, 3> J3;
    J3.setRandom();
    jc.add(&dv3, J3);
    DummyDesignVariable<4> dv4;
    dv4.setBlockIndex(4);
    dv4.setActive(true);
    Eigen::Matrix<double, 2, 4> J4;
    J4.setRandom();
    jc.add(&dv4, J4);
    ASSERT_EQ(2, jc.rows());
    // Now apply the chain rule.
    Eigen::Matrix<double, 5, 2> H;
    H.setRandom();
    jc.applyChainRule(H);
    ASSERT_EQ(5, jc.rows());
    sm::eigen::assertEqual(H * J1, jc.Jacobian(&dv1), SM_SOURCE_FILE_POS, "Checking for correct application of the chain rule");
    sm::eigen::assertEqual(H * J2, jc.Jacobian(&dv2), SM_SOURCE_FILE_POS, "Checking for correct application of the chain rule");
    sm::eigen::assertEqual(H * J3, jc.Jacobian(&dv3), SM_SOURCE_FILE_POS, "Checking for correct application of the chain rule");
    sm::eigen::assertEqual(H * J4, jc.Jacobian(&dv4), SM_SOURCE_FILE_POS, "Checking for correct application of the chain rule");

    JacobianContainerSparse<> jc1(2);
    JacobianContainerSparse<> jc2(2);
    jc1.add(&dv1, J1);
    jc1.add(&dv2, J2);
    Eigen::Matrix<double, 2, 2> C;
    jc2.add(jc1, &C); // add with chain rule
    jc1.applyChainRule(C);
    sm::eigen::assertEqual(jc1.asDenseMatrix(), jc2.asDenseMatrix(), SM_SOURCE_FILE_POS, "Checking for correct application of the chain rule");

  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}

TEST(JacobianContainerTests, testAddContainers)
{
  try {
    using namespace aslam::backend;
    JacobianContainerSparse<> jc1(2);
    JacobianContainerSparse<> jc2(2);
    // jc3 is the odd man out.
    JacobianContainerSparse<> jc3(3);
    DummyDesignVariable<1> dv1;
    dv1.setBlockIndex(1);
    dv1.setActive(true);
    Eigen::Matrix<double, 2, 1> J1;
    J1.setRandom();
    jc1.add(&dv1, J1);
    DummyDesignVariable<2> dv2;
    dv2.setBlockIndex(2);
    dv2.setActive(true);
    Eigen::Matrix<double, 2, 2> J2;
    J2.setRandom();
    jc1.add(&dv2, J2);
    /// dv2 is in both containers, jc1 and jc2!
    jc2.add(&dv2, J2);
    DummyDesignVariable<3> dv3;
    dv3.setBlockIndex(5);
    dv3.setActive(true);
    Eigen::Matrix<double, 2, 3> J3;
    J3.setRandom();
    jc2.add(&dv3, J3);
    DummyDesignVariable<4> dv4;
    dv4.setBlockIndex(0);
    dv4.setActive(true);
    Eigen::Matrix<double, 2, 4> J4;
    J4.setRandom();
    jc2.add(&dv4, J4);
    DummyDesignVariable<5> dv5;
    dv5.setBlockIndex(5);
    dv5.setActive(true);
    Eigen::Matrix<double, 3, 5> J5;
    J5.setRandom();
    jc3.add(&dv5, J5);
    ASSERT_THROW(jc1.add(jc3), JacobianContainerSparse<>::Exception) << "Incompatible row sizes";
    ASSERT_THROW(jc2.add(jc3), JacobianContainerSparse<>::Exception) << "Incompatible row sizes";
    ASSERT_THROW(jc3.add(jc1), JacobianContainerSparse<>::Exception) << "Incompatible row sizes";
    ASSERT_THROW(jc3.add(jc2), JacobianContainerSparse<>::Exception) << "Incompatible row sizes";
    ASSERT_EQ(2u, jc1.numDesignVariables());
    ASSERT_EQ(3u, jc2.numDesignVariables());
    /// After adding, jc1 should have 4 design variables because jc2 is in both containers.
    JacobianContainerSparse<> jc1PlusJc2 = jc1;
    jc1PlusJc2.add(jc2);
    ASSERT_EQ(4u, jc1PlusJc2.numDesignVariables());
    sm::eigen::assertEqual(jc1PlusJc2.Jacobian(&dv1), J1, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    sm::eigen::assertEqual(jc1PlusJc2.Jacobian(&dv2), J2 + J2, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    sm::eigen::assertEqual(jc1PlusJc2.Jacobian(&dv3), J3, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    sm::eigen::assertEqual(jc1PlusJc2.Jacobian(&dv4), J4, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    /// Check that addition is symmetric
    JacobianContainerSparse<> jc2PlusJc1 = jc2;
    jc2PlusJc1.add(jc1);
    ASSERT_EQ(4u, jc2PlusJc1.numDesignVariables());
    sm::eigen::assertEqual(jc2PlusJc1.Jacobian(&dv1), J1, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    sm::eigen::assertEqual(jc2PlusJc1.Jacobian(&dv2), J2 + J2, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    sm::eigen::assertEqual(jc2PlusJc1.Jacobian(&dv3), J3, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
    sm::eigen::assertEqual(jc2PlusJc1.Jacobian(&dv4), J4, SM_SOURCE_FILE_POS, "Checking for correct Jacobians after add");
  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}

TEST(JacobianContainerTests, testBuildHessian)
{
  try {
    using namespace aslam::backend;
    JacobianContainerSparse<> jc1(2);
    JacobianContainerSparse<> jc2(2);
    DummyDesignVariable<3> dv1;
    dv1.setBlockIndex(0);
    dv1.setActive(true);
    Eigen::Matrix<double, 2, 3> J1;
    J1.setRandom();
    jc1.add(&dv1, J1);
    DummyDesignVariable<2> dv2;
    dv2.setBlockIndex(1);
    dv2.setActive(true);
    Eigen::Matrix<double, 2, 2> J2;
    J2.setRandom();
    jc1.add(&dv2, J2);
    // Create the block index list to init
    // the sparse matrix.d
    std::vector<int> bi;
    bi.push_back(3);
    bi.push_back(2);
    // bi = cumsum(bi)
    std::partial_sum(bi.begin(), bi.end(), bi.begin());
    // Initialize the block matrix with the correct block structure.
    sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Hessian(bi, bi);
    Eigen::VectorXd rhs(bi[bi.size() - 1]);
    rhs.setZero();
    Eigen::VectorXd e(2);
    e.setRandom();
    Eigen::MatrixXd invR = sm::eigen::randomCovariance<2>() * 100;
    Eigen::MatrixXd sqrtInvR;
    sm::eigen::computeMatrixSqrt(invR, sqrtInvR);
    ASSERT_EQ(e.size(), sqrtInvR.rows());
    ASSERT_EQ(e.size(), sqrtInvR.cols());
    ASSERT_EQ(invR.rows(), sqrtInvR.rows());
    Eigen::MatrixXd invRReconstructed = sqrtInvR * sqrtInvR.transpose();
    ASSERT_DOUBLE_MX_EQ(invR, invRReconstructed, 1e-9, "Reconstruct covariance");
    ASSERT_EQ(2u, jc1.numDesignVariables());
    jc1.evaluateHessian(e, sqrtInvR, Hessian, rhs);
    /// Check that there are blocks filled in:
    ASSERT_FALSE(Hessian.isBlockSet(1, 0));
    ASSERT_TRUE(Hessian.isBlockSet(0, 0));
    ASSERT_TRUE(Hessian.isBlockSet(0, 1));
    ASSERT_TRUE(Hessian.isBlockSet(1, 1));
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(0, 0), J1.transpose() * invR * J1, 1e-9, "Block 0,0");
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(0, 1), J1.transpose() * invR * J2, 1e-9, "Block 0,1");
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(1, 1), J2.transpose() * invR * J2, 1e-9, "Block 1,1");
    ASSERT_DOUBLE_MX_EQ(rhs.segment(0, J1.cols()), -J1.transpose() * invR * e, 1e-9, "Block 0");
    ASSERT_DOUBLE_MX_EQ(rhs.segment(Hessian.rowBaseOfBlock(1), J2.cols()), -J2.transpose() * invR * e, 1e-9, "Block 1");
  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}


TEST(JacobianContainerTests, testBuildHessian2)
{
  try {
    using namespace aslam::backend;
    JacobianContainerSparse<> jc1(2);
    DummyDesignVariable<3> dv1;
    dv1.setBlockIndex(0);
    dv1.setActive(true);
    Eigen::Matrix<double, 2, 3> J1;
    J1.setRandom();
    jc1.add(&dv1, J1);
    DummyDesignVariable<2> dv2;
    dv2.setBlockIndex(1);
    dv2.setActive(true);
    Eigen::Matrix<double, 2, 2> J2;
    J2.setRandom();
    jc1.add(&dv2, J2);
    // Create the block index list to init
    // the sparse matrix.d
    std::vector<int> bi;
    bi.push_back(3);
    bi.push_back(2);
    // bi = cumsum(bi)
    std::partial_sum(bi.begin(), bi.end(), bi.begin());
    // Initialize the block matrix with the correct block structure.
    sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Hessian(bi, bi);
    Eigen::VectorXd rhs(bi[bi.size() - 1]);
    rhs.setZero();
    Eigen::VectorXd e(2);
    e.setRandom();
    Eigen::MatrixXd invR = sm::eigen::randomCovariance<2>();
    Eigen::MatrixXd sqrtInvR;
    sm::eigen::computeMatrixSqrt(invR, sqrtInvR);
    Eigen::MatrixXd invRReconstructed = sqrtInvR * sqrtInvR.transpose();
    ASSERT_DOUBLE_MX_EQ(invR, invRReconstructed, 1e-9, "Reconstruct covariance");
    ASSERT_EQ(2u, jc1.numDesignVariables());
    jc1.evaluateHessian(e, sqrtInvR, Hessian, rhs);
    /// Check that there are blocks filled in:
    ASSERT_FALSE(Hessian.isBlockSet(1, 0));
    ASSERT_TRUE(Hessian.isBlockSet(0, 0));
    ASSERT_TRUE(Hessian.isBlockSet(0, 1));
    ASSERT_TRUE(Hessian.isBlockSet(1, 1));
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(0, 0),  J1.transpose() * invR * J1, 1e-9, "Block 0,0");
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(0, 1),  J1.transpose() * invR * J2, 1e-9, "Block 0,1");
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(1, 1),  J2.transpose() * invR * J2, 1e-9, "Block 1,1");
    ASSERT_DOUBLE_MX_EQ(rhs.segment(0, J1.cols()), - J1.transpose() * invR * e, 1e-9, "Block 0");
    ASSERT_DOUBLE_MX_EQ(rhs.segment(Hessian.rowBaseOfBlock(1), J2.cols()), - J2.transpose() * invR * e, 1e-9, "Block 1");
    // Do this twice! I should get double the values
    jc1.evaluateHessian(e, sqrtInvR, Hessian, rhs);
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(0, 0), 2.0 * J1.transpose() * invR * J1, 1e-9, "Block 0,0");
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(0, 1), 2.0 * J1.transpose() * invR * J2, 1e-9, "Block 0,1");
    ASSERT_DOUBLE_MX_EQ(*Hessian.block(1, 1), 2.0 * J2.transpose() * invR * J2, 1e-9, "Block 1,1");
    ASSERT_DOUBLE_MX_EQ(rhs.segment(0, J1.cols()), -2.0 * J1.transpose() * invR * e, 1e-9, "Block 0");
    ASSERT_DOUBLE_MX_EQ(rhs.segment(Hessian.rowBaseOfBlock(1), J2.cols()), -2.0 * J2.transpose() * invR * e, 1e-9, "Block 1");
  } catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}

TYPED_TEST(JacobianContainerTests, testIsFinite)
{
  try {

    using namespace aslam::backend;
    DummyDesignVariable<1> dv1, dv2;
    dv1.setBlockIndex(0);
    dv1.setColumnBase(0);
    dv1.setActive(true);
    dv2.setBlockIndex(1);
    dv2.setColumnBase(dv1.minimalDimensions());
    dv2.setActive(true);
    Eigen::Matrix<double, 2, 1> J;
    J.setRandom();

    boost::shared_ptr<TypeParam> jc = this->getJacobianContainer(2, dv1.minimalDimensions() + dv2.minimalDimensions());
    jc->add(&dv1, J);
    jc->add(&dv2, J);
    EXPECT_TRUE(jc->isFinite(static_cast<const DesignVariable&>(dv1)));
    EXPECT_TRUE(jc->isFinite(static_cast<const DesignVariable&>(dv2)));

    J(0,0) = std::numeric_limits<double>::signaling_NaN();
    jc->add(&dv1, J);
    EXPECT_FALSE(jc->isFinite(static_cast<const DesignVariable&>(dv1)));
    EXPECT_TRUE(jc->isFinite(static_cast<const DesignVariable&>(dv2)));

  }  catch (const std::exception& e) {
    FAIL() << "Exception: " << e.what();
  }
}
