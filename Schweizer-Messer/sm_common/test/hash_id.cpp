#include <algorithm>
#include <unordered_set>

#include <gtest/gtest.h>

#include "sm/hash_id.hpp"

using namespace sm;

TEST(SmCommonTestSuite, hashId_different) {
  HashId a(HashId::random()), b(HashId::random());
  EXPECT_NE(a, b);
}

TEST(SmCommonTestSuite, hashId_validity) {
  HashId a, b;
  EXPECT_FALSE(a.isValid());
  EXPECT_FALSE(b.isValid());
  EXPECT_EQ(a,b);
  a.randomize();
  EXPECT_TRUE(a.isValid());
}

TEST(SmCommonTestSuite, hashId_string) {
  HashId a(HashId::random()), b(HashId::random());
  std::string as(a.hexString()), bs(b.hexString());
  EXPECT_NE(as, bs);
  EXPECT_EQ(as.length(), 32);
  EXPECT_EQ(bs.length(), 32);
}

TEST(SmCommonTestSuite, hashId_deserialize) {
  HashId a;
  std::string as(a.hexString());
  HashId b;
  EXPECT_TRUE(b.fromHexString(as));
  EXPECT_EQ(a, b);
}

TEST(SmCommonTestSuite, hashId_stdHash) {
  std::unordered_set<HashId> hashes;
  HashId needle(HashId::random());
  hashes.insert(needle);
  hashes.insert(HashId::random());
  std::unordered_set<HashId>::iterator found = hashes.find(needle);
  EXPECT_TRUE(found != hashes.end());
  EXPECT_EQ(*found, needle);
}
