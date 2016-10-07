/*
 * MatrixStackTest.cpp
 *
 *  Created on: 21.03.2016
 *      Author: Ulrich Schwesinger
 */

#include <sm/eigen/gtest.hpp>

#undef NDEBUG

#include <aslam/backend/MatrixStack.hpp>


TEST(MatrixStackTestSuites, testMatrixStack)
{
  try
  {
    using namespace aslam::backend;
    const int numRows = 3;

    // Construct the stack too small to see if dynamic memory allocation works
    MatrixStack stack(numRows, 1, 8);

    // Check it is really empty
    {
      SCOPED_TRACE("Testing empty stack");
      EXPECT_TRUE(stack.empty());
      EXPECT_EQ(0, stack.numElements());
      EXPECT_EQ(0, stack.numMatrices());
      EXPECT_ANY_THROW(stack.top());
    }

    // Test pushing a matrix with invalid number of rows
    {
      SCOPED_TRACE("Testing push() with invalid number of rows");
      const auto M = Eigen::Matrix<double, numRows + 1, 3>::Random().eval();
      EXPECT_ANY_THROW(stack.push(M));
    }

    // Test push scalar and pop guard
    {
      SCOPED_TRACE("Testing push() with scalar");
      const double scalar = 0.1;
      const auto pg = stack.pushWithGuard(scalar);
      sm::eigen::assertEqual(scalar*Eigen::MatrixXd::Identity(numRows, numRows), stack.top(), SM_SOURCE_FILE_POS, "Testing push() with scalar");
    }

    // Destructor of pop guard should have popped scalar
    EXPECT_TRUE(stack.empty());

    // Push() and top() should return the same matrix again
    {
      SCOPED_TRACE("Testing push() and top() after initial construction");
      const auto M = Eigen::Matrix<double, numRows, 3>::Random().eval();
      stack.push(M);
      EXPECT_EQ(M.size(), stack.numElements());
      EXPECT_EQ(1, stack.numMatrices());
      sm::eigen::assertEqual(M, stack.top(), SM_SOURCE_FILE_POS, "Testing push() and top() after initial construction");
    }

    // Pop() should yield empty stack again
    {
      SCOPED_TRACE("Testing pop()");
      stack.pop();
      EXPECT_EQ(0, stack.numElements());
      EXPECT_EQ(0, stack.numMatrices());
      EXPECT_TRUE(stack.empty());
      EXPECT_ANY_THROW(stack.top());
    }

    // Push() and top() should return the same matrix again
    {
      SCOPED_TRACE("Testing push() and top() after pop() was called");
      const auto M = Eigen::Matrix<double, numRows, 4>::Random().eval();
      stack.push(M);
      EXPECT_EQ(M.size(), stack.numElements());
      EXPECT_EQ(1, stack.numMatrices());
      sm::eigen::assertEqual(M, stack.top(), SM_SOURCE_FILE_POS, "Testing push() and top() after pop() was called");
    }

    // Test that pop() yields intermediate matrices again
    {
      SCOPED_TRACE("Testing that pop() yields intermediate matrices again");

      while(!stack.empty()) // clear stack
        stack.pop();

      std::vector< Eigen::Matrix<double, numRows, numRows>, Eigen::aligned_allocator< Eigen::Matrix<double, numRows, numRows> > > storage;
      storage.push_back(Eigen::Matrix<double, numRows, numRows>::Identity());

      for (size_t i=0; i<10; ++i) {
        const auto M = Eigen::Matrix<double, numRows, numRows>::Random().eval();
        stack.push(M);
        storage.push_back( storage.back()*M );
      }

      while(!stack.empty()) {
        sm::eigen::assertEqual(storage.back(), stack.top(), SM_SOURCE_FILE_POS, "Testing push() and top() after pop() was called");
        stack.pop();
        storage.pop_back();
      }
    }

    // Push() two matrices should keep the product in the top entry
    {
      SCOPED_TRACE("Testing push() of multiple matrices");

      while(!stack.empty()) // clear stack
        stack.pop();

      const auto M1 = Eigen::Matrix<double, numRows, numRows>::Random().eval();
      EXPECT_NO_THROW(stack.push(M1));
      sm::eigen::assertEqual(M1, stack.top(), SM_SOURCE_FILE_POS, "Testing push() of multiple matrices");

      const auto M2 = Eigen::Matrix<double, numRows + 1, numRows>::Random().eval();
      EXPECT_ANY_THROW(stack.push(M2)); // incompatible matrix sizes
      sm::eigen::assertEqual(M1, stack.top(), SM_SOURCE_FILE_POS, "Testing push() of multiple matrices");

      const auto M3 = Eigen::Matrix<double, numRows, numRows>::Random().eval();
      Eigen::MatrixXd expected = M1;
      for (size_t i=0; i<10; ++i) {
        EXPECT_NO_THROW(stack.push(M3));
        expected = expected*M3;
        sm::eigen::assertEqual(expected, stack.top(), SM_SOURCE_FILE_POS, "Testing push() of multiple matrices");
      }
    }
  }
  catch(std::exception const & e)
  {
    FAIL() << e.what();
  }

} /* TEST(MatrixStackTestSuites, testMatrixStack) */
