#include <gtest/gtest.h>
#include <limits>
#include <sm/assert_macros.hpp>

TEST(SmCommonTestSuite,testAssertMacros) 
{
  
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
  
  {
    double* val = new double;
    EXPECT_NO_THROW( SM_ASSERT_TRUE(Exception, true, "") );
    EXPECT_NO_THROW( SM_ASSERT_FALSE(Exception, false, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_LT(Exception, 0.0, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GT_LE(Exception, 0.1, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_LE(Exception, 0.0, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_LE(Exception, 1.0, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_LT(Exception, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GT(Exception, 1.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_POSITIVE(Exception, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NONNEGATIVE(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NEGATIVE(Exception, -1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NONPOSITIVE(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_ZERO(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NOTNULL(Exception, val, "") );
    EXPECT_NO_THROW( SM_ASSERT_LE(Exception, 0.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE(Exception, 0.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NE(Exception, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_EQ(Exception, 0.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NEAR(Exception, 0.0, 1.0, 2.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_FINITE(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NOTNAN(Exception, 0.0, "") );
    delete val;
  }
    
  {
    double* val = NULL;
    EXPECT_THROW( SM_ASSERT_TRUE(Exception, false, ""), Exception);
    EXPECT_THROW( SM_ASSERT_FALSE(Exception, true, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_LT(Exception, 1.0, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GT_LE(Exception, 0.0, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_LE(Exception, -0.1, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_LE(Exception, 1.1, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_LT(Exception, 1.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GT(Exception, 0.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_POSITIVE(Exception, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NONNEGATIVE(Exception, -1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NEGATIVE(Exception, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NONPOSITIVE(Exception, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_ZERO(Exception, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NOTNULL(Exception, val, ""), Exception );
    EXPECT_THROW( SM_ASSERT_LE(Exception, 1.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE(Exception, -1.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NE(Exception, 0.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_EQ(Exception, 1.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NEAR(Exception, 0.0, 1.0, 0.5, ""), Exception );
    EXPECT_THROW( SM_ASSERT_FINITE(Exception, std::numeric_limits<float>::infinity(), ""), Exception );
    EXPECT_THROW( SM_ASSERT_NOTNAN(Exception, std::numeric_limits<float>::signaling_NaN(), ""), Exception );
  }
}
