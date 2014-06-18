/*
 * NodeDistributedCacheTests.cpp
 *
 *  Created on: Oct 4, 2012
 *      Author: hannes
 */
#include "gtest/gtest.h"
#include "boost/typeof/typeof.hpp"
#include "bsplines/NodeDistributedCache.hpp"
#include <sm/assert_macros.hpp>

using namespace nodecache;

struct TestNode {
	NodeDistributedCache<TestNode>::PerNodeCache cache;
public:
	NodeDistributedCache<TestNode>::PerNodeCache & accessCache(){
		return cache;
	}
};

struct TestValue{
	int i;

	TestValue(int i = 0) : i (i){
	}

	TestValue(const TestValue & v) : i (v.i){
	}

	~TestValue(){
	}
};

typedef NodeDistributedCache<TestNode> TestCache;

TEST(NodeDistributedCacheTestSuite, testInitialization)
{
	TestCache cache;

	{
		BOOST_AUTO(testSlot, cache.registerCacheableValue<TestValue>());

		TestNode testNode;

		int val = 3238;
		testSlot->accessValue(testNode) = TestValue(val);

		SM_ASSERT_EQ(std::runtime_error, testSlot->accessValue(testNode).i , val, "");
	}
}
