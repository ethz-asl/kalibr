#include <gtest/gtest.h>

#include <iostream>
#include <sm/timing/Timer.hpp>

/// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto rval = RUN_ALL_TESTS();
#ifdef aslam_backend_ENABLE_TIMING
  sm::timing::Timing::print(std::cout, sm::timing::SORT_BY_TOTAL);
#endif
  return rval;
}

