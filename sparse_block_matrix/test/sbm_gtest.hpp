/**
 * @file   sbm_gtest.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Wed Jan 11 09:29:57 2012
 * 
 * @brief  Helper functions for unit testing SBM
 * 
 * 
 */
#ifndef _SBM_GTEST_H_
#define _SBM_GTEST_H_



namespace sparse_block_matrix {
    template<typename MATRIX1_TYPE, typename MATRIX2_TYPE, typename T>
    void expectNear(const MATRIX1_TYPE & A, const MATRIX2_TYPE & B, T tolerance, std::string const & message = "")
    {
      // These assert statements will return from this function but not from the base unit test.
      ASSERT_EQ(A.rows(),B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same size";
      ASSERT_EQ(A.cols(),B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same size";

      for(int r = 0; r < A.rows(); r++)
	{
           for(int c = 0; c < A.cols(); c++)
             {
               ASSERT_NEAR(A(r,c),B(r,c),tolerance) << message << "\nTolerance comparison failed at (" << r << "," << c << ")"
						    << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
             }
	}
    }

    template<typename MATRIX1_TYPE, typename MATRIX2_TYPE>
    void expectEqual(const MATRIX1_TYPE & A, const MATRIX2_TYPE & B, std::string const & message = "")
    {
      // These assert statements will return from this function but not from the base unit test.
      ASSERT_EQ(B.rows(),A.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same size";
      ASSERT_EQ(B.cols(),A.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same size";

      for(int r = 0; r < A.rows(); r++)
	{
           for(int c = 0; c < A.cols(); c++)
             {
               ASSERT_EQ(B(r,c),A(r,c)) << message << "\nEquality comparison failed at (" << r << "," << c << ")"
						    << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
             }
	}
    }


} // namespace sbm

#endif /* _SBM_GTEST_H_ */
