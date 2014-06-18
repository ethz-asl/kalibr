#include <gtest/gtest.h>
#include <sm/timing/NsecTimeUtilities.hpp>

TEST( NsetTimeTestSuite, testChronoConversion ) {

  std::chrono::system_clock::time_point tp1 = std::chrono::system_clock::now();
  sm::timing::NsecTime ns1 = sm::timing::chronoToNsec( tp1 );
  std::chrono::system_clock::time_point tp2 = sm::timing::nsecToChrono( ns1 );
  ASSERT_TRUE(tp1 == tp2);
  
}


TEST( NsetTimeTestSuite, testSecConversion ) {

  sm::timing::NsecTime ns1 = sm::timing::nsecNow();
  double s2 = sm::timing::nsecToSec(ns1);
  sm::timing::NsecTime ns2 = sm::timing::secToNsec(s2);
  
  ASSERT_LT(abs(ns1-ns2), 1000000);
  
}
