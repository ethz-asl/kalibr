#include <gtest/gtest.h>
#include <sm/BoostPropertyTree.hpp>
#include <boost/property_tree/xml_parser.hpp>

TEST(PTreeTestSuite, testBoostPTree)
{

  sm::BoostPropertyTree wbpt;
  
  wbpt.setDouble("d",0.1);
  wbpt.setDouble("d/d",0.2);
  wbpt.setInt("i",1);
  wbpt.setInt("i/i",2);
  wbpt.setBool("b",true);
  wbpt.setBool("b/b", false);
  wbpt.setString("/s","hello");
  wbpt.setString("s/s","goodbye");
  wbpt.saveXml("test.xml");

  try 
    {
      sm::BoostPropertyTree pt;
      
      pt.loadXml("test.xml");
      
      ASSERT_NEAR(pt.getDouble("d"), 0.1, 1e-16);
      ASSERT_NEAR(pt.getDouble("/d"), 0.1, 1e-16);
      ASSERT_NEAR(pt.getDouble("d/d"), 0.2, 1e-16);
      ASSERT_NEAR(pt.getDouble("/d/d"), 0.2, 1e-16);
      // Push a namespace on to the stack.
      sm::PropertyTree dpt(pt,"d");
      ASSERT_NEAR(dpt.getDouble("d"), 0.2, 1e-16);
      ASSERT_NEAR(dpt.getDouble("/d"), 0.1, 1e-16);
      ASSERT_NEAR(dpt.getDouble("/d/d"), 0.2, 1e-16);

      ASSERT_EQ(pt.getInt("i"), 1);
      ASSERT_EQ(pt.getInt("/i"), 1);
      ASSERT_EQ(pt.getInt("i/i"), 2);
      ASSERT_EQ(pt.getInt("/i/i"), 2);
      // Push a namespace on to the stack.
      sm::PropertyTree ipt(pt,"i");
      ASSERT_EQ(ipt.getInt("i"), 2);
      ASSERT_EQ(ipt.getInt("/i"), 1);
      ASSERT_EQ(ipt.getInt("/i/i"), 2);

      ASSERT_EQ(pt.getBool("b"), true);
      ASSERT_EQ(pt.getBool("/b"), true);
      ASSERT_EQ(pt.getBool("b/b"), false);
      ASSERT_EQ(pt.getBool("/b/b"), false);
      // Push a namespace on to the stack.
      sm::PropertyTree bpt(pt,"b");
      ASSERT_EQ(bpt.getBool("b"), false);
      ASSERT_EQ(bpt.getBool("/b"), true);
      ASSERT_EQ(bpt.getBool("/b/b"), false); 

      
      ASSERT_EQ(pt.getString("s"), std::string("hello"));
      ASSERT_EQ(pt.getString("/s"), std::string("hello"));
      ASSERT_EQ(pt.getString("s/s"), std::string("goodbye"));
      ASSERT_EQ(pt.getString("/s/s"), std::string("goodbye"));
      // Push a namespace on to the stack.
      sm::PropertyTree spt(pt,"s");
      ASSERT_EQ(spt.getString("s"), std::string("goodbye"));
      ASSERT_EQ(spt.getString("/s"), std::string("hello"));
      ASSERT_EQ(spt.getString("/s/s"), std::string("goodbye"));
    }
  catch(const std::exception & e)
    {
      FAIL() << "Unhandled exception: " << e.what();
    }
}

TEST(PTreeTestSuite, testFindFile)
{
  // Not finding should throw
  EXPECT_THROW(sm::findFile("BoostPropertyTreeImplementation.cpp", "RIDICULOUS_ENVIRONMENT_VARIABLE_THAT_CANNOT_EXIST"), std::runtime_error);
  EXPECT_THROW(sm::findFile("RidiculousFilenameThatCannotExist", "ROS_PACKAGE_PATH"), std::runtime_error);

  // Let's test whether we can find a file
  EXPECT_NO_THROW(sm::findFile(".", "PWD"));
}
