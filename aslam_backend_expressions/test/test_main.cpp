#include <gtest/gtest.h>

//#define TEST_WITH_RANDOM_SEED
#ifdef TEST_WITH_RANDOM_SEED
#include <sm/random.hpp>
#endif

/// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
#ifdef TEST_WITH_RANDOM_SEED
  sm::random::seed(std::time(nullptr));
#endif
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
