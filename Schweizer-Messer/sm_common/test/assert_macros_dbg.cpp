#include <gtest/gtest.h>

#undef NDEBUG
#include <sm/assert_macros.hpp>

TEST(SmCommonTestSuite,testAssertMacrosDbg)
{
  
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  {
    double* val = new double;
    EXPECT_NO_THROW( SM_ASSERT_TRUE_DBG(Exception, true, "") );
    EXPECT_NO_THROW( SM_ASSERT_FALSE_DBG(Exception, false, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_LT_DBG(Exception, 0.0, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GT_LE_DBG(Exception, 0.1, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_LE_DBG(Exception, 0.0, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_LE_DBG(Exception, 1.0, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_LT_DBG(Exception, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GT_DBG(Exception, 1.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_POSITIVE_DBG(Exception, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NONNEGATIVE_DBG(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NEGATIVE_DBG(Exception, -1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NONPOSITIVE_DBG(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_ZERO_DBG(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NOTNULL_DBG(Exception, val, "") );
    EXPECT_NO_THROW( SM_ASSERT_LE_DBG(Exception, 0.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_GE_DBG(Exception, 0.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NE_DBG(Exception, 0.0, 1.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_EQ_DBG(Exception, 0.0, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NEAR_DBG(Exception, 0.0, 1.0, 2.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_FINITE_DBG(Exception, 0.0, "") );
    EXPECT_NO_THROW( SM_ASSERT_NOTNAN_DBG(Exception, 0.0, "") );
    delete val;
  }
  
  {
    double* val = NULL;
    EXPECT_THROW( SM_ASSERT_TRUE_DBG(Exception, false, ""), Exception);
    EXPECT_THROW( SM_ASSERT_FALSE_DBG(Exception, true, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_LT_DBG(Exception, 1.0, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GT_LE_DBG(Exception, 0.0, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_LE_DBG(Exception, -0.1, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_LE_DBG(Exception, 1.1, 0.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_LT_DBG(Exception, 1.0, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GT_DBG(Exception, 0.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_POSITIVE_DBG(Exception, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NONNEGATIVE_DBG(Exception, -1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NEGATIVE_DBG(Exception, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NONPOSITIVE_DBG(Exception, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_ZERO_DBG(Exception, 1.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NOTNULL_DBG(Exception, val, ""), Exception );
    EXPECT_THROW( SM_ASSERT_LE_DBG(Exception, 1.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_GE_DBG(Exception, -1.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NE_DBG(Exception, 0.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_EQ_DBG(Exception, 1.0, 0.0, ""), Exception );
    EXPECT_THROW( SM_ASSERT_NEAR_DBG(Exception, 0.0, 1.0, 0.5, ""), Exception );
    EXPECT_THROW( SM_ASSERT_FINITE_DBG(Exception, std::numeric_limits<float>::infinity(), ""), Exception  );
    EXPECT_THROW( SM_ASSERT_NOTNAN_DBG(Exception, std::numeric_limits<float>::signaling_NaN(), ""), Exception  );

  }
}
