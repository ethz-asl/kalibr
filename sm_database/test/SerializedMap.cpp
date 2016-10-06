#include <gtest/gtest.h>
#include <sm/database/SerializedMap.hpp>
#include <sm/boost/serialization.hpp>


using namespace sm::database;
struct tester
{
  int a, b;
  tester() : a(rand()), b(rand()){}
  bool isBinaryEqual(const tester & t2)
  {
    return a==t2.a && b == t2.b;
  }

  template<typename Archive>
  void serialize(Archive & ar, const unsigned int /* version */)
  {
    ar & a;
    ar & b;
  }

};

TEST(SmDatabaseTestSuite, testCreateDb)
{
  boost::filesystem::path testdb("test.db");
  try {
    
    if( boost::filesystem::exists(testdb) )
      {
	boost::filesystem::remove(testdb);
      }
    
    
    {
        sm::database::SerializedMap<tester, PortableBinaryArchive> smap(testdb, "tester");
    }
    
    ASSERT_TRUE(boost::filesystem::exists(testdb));

    }
  catch(const std::exception & e)
    {
      FAIL() << "Uncaught exception: " << e.what() << std::endl;
    }

  
}


TEST(SmDatabaseTestSuite, testSerializedMap)
{
  boost::filesystem::path testdb("test.db");
  try {
    
    if( boost::filesystem::exists(testdb) )
      {
	boost::filesystem::remove(testdb);
      }
    
    
    sm::database::SerializedMap<tester, PortableBinaryArchive> smap(testdb, "tester");
    ASSERT_EQ(smap.tableName(), "tester");
    //--gtest_catch_exceptions
    ASSERT_THROW(smap.get(1), sm::database::SqlException);
    for(int i = 0; i < 100; i++)
      {
	boost::shared_ptr<tester> t1(new tester), t2;
	// This tests overwrite
	smap.set(1,t1);
	//std::cout << "a: " << t1->a << ", b: " << t1->b << std::endl;
	t2 = smap.get(1);
	ASSERT_TRUE(t1->isBinaryEqual(*t1));
	ASSERT_TRUE(t1->isBinaryEqual(*t2)) << "Iteration " << i;
      }
  }
  catch(const std::exception & e)
    {
      FAIL() << "Uncaught exception: " << e.what() << std::endl;
    }
  //boost::filesystem::remove(testdb);
}


TEST(SmDatabaseTestSuite, testSerializedMapPersistence)
{
  boost::filesystem::path testdb("test.db");
  try {
    
    if( boost::filesystem::exists(testdb) )
      {
	boost::filesystem::remove(testdb);
      }


    std::vector< boost::shared_ptr<tester> > vals;
    
    {   // Scope block so that smap will go out of scope. 
        sm::database::SerializedMap<tester, PortableBinaryArchive> smap(testdb, "tester");
      


      for(int i = 0; i < 100; i++)
	{
	  ASSERT_THROW(smap.get(i), sm::database::SqlException);
	}
      
      for(int i = 0; i < 100; i++)
	{
	  boost::shared_ptr<tester> t1(new tester), t2;
	  vals.push_back(t1);
	  smap.set(i,t1);
	  t2 = smap.get(i);
	  ASSERT_TRUE(t1->isBinaryEqual(*t1));
	  ASSERT_TRUE(t1->isBinaryEqual(*t2));
	}
    } // end of scope block. smap is out of scope and we can re-open the database


    {   // Scope block so that smap will go out of scope. 
        sm::database::SerializedMap<tester, PortableBinaryArchive> smap(testdb, "tester");
            
      for(int i = 0; i < 100; i++)
	{
	  boost::shared_ptr<tester> t2;
	  ASSERT_NO_THROW(t2 = smap.get(i)) << "Item " << i;
	  ASSERT_TRUE(t2->isBinaryEqual(*vals[i]));
	}
    } // end of scope block. smap is out of scope

	
   

  }
  catch(const std::exception & e)
    {
      FAIL() << "Uncaught exception: " << e.what() << std::endl;
    }
 
}
