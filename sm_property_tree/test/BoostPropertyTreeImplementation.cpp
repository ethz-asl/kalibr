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

  EXPECT_TRUE(wbpt.doesKeyExist(""));
  EXPECT_TRUE(wbpt.doesKeyExist("d"));
  EXPECT_TRUE(wbpt.doesKeyExist("d/d"));
  EXPECT_TRUE(wbpt.doesKeyExist("i"));
  EXPECT_TRUE(wbpt.doesKeyExist("i/i"));
  EXPECT_TRUE(wbpt.doesKeyExist("b"));
  EXPECT_TRUE(wbpt.doesKeyExist("b/b"));
  EXPECT_TRUE(wbpt.doesKeyExist("s"));
  EXPECT_TRUE(wbpt.doesKeyExist("s/s"));
  EXPECT_FALSE(wbpt.doesKeyExist("xyz"));

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
      ASSERT_EQ(ipt.getInt(""), 1);
      ASSERT_EQ(ipt.getInt("i"), 2);
      ASSERT_EQ(ipt.getInt("/i"), 1);
      ASSERT_EQ(ipt.getInt("/i/i"), 2);

      ASSERT_EQ(pt.getBool("b"), true);
      ASSERT_EQ(pt.getBool("/b"), true);
      ASSERT_EQ(pt.getBool("b/b"), false);
      ASSERT_EQ(pt.getBool("/b/b"), false);
      // Push a namespace on to the stack.
      sm::PropertyTree bpt(pt,"b");
      ASSERT_EQ(bpt.getBool(""), true);
      ASSERT_EQ(bpt.getBool("b"), false);
      ASSERT_EQ(bpt.getBool("/b"), true);
      ASSERT_EQ(bpt.getBool("/b/b"), false); 

      
      ASSERT_EQ(pt.getString("s"), std::string("hello"));
      ASSERT_EQ(pt.getString("/s"), std::string("hello"));
      ASSERT_EQ(pt.getString("s/s"), std::string("goodbye"));
      ASSERT_EQ(pt.getString("/s/s"), std::string("goodbye"));
      // Push a namespace on to the stack.
      sm::PropertyTree spt(pt,"s");
      ASSERT_EQ(spt.getString(""), std::string("hello"));
      ASSERT_EQ(spt.getString("s"), std::string("goodbye"));
      ASSERT_EQ(spt.getString("/s"), std::string("hello"));
      ASSERT_EQ(spt.getString("/s/s"), std::string("goodbye"));


      {
        std::vector<const char *> expectedKeys{"d", "i", "b", "s"};
        int i = 0;
        for (auto & c : pt.getChildren()){
          EXPECT_EQ(expectedKeys[i++], c.key);
          if(c.key == "d"){
            ASSERT_LT(i, expectedKeys.size());
            EXPECT_NEAR(0.2, c.pt.getDouble("d"), 1e-16);
          }
          if(c.key == "s"){
            std::vector<const char *> expectedKeys{"s"};
            int i = 0;
            for (auto & c2 : c.pt.getChildren()){
              ASSERT_LT(i, expectedKeys.size());
              EXPECT_EQ(expectedKeys[i++], c2.key);
            }
            EXPECT_EQ(expectedKeys.size(), i);
          }
        }
        EXPECT_EQ(expectedKeys.size(), i);
      }
      {
        std::vector<const char *> expectedKeys{"s"};
        int i = 0;
        for (auto & c2 : sm::PropertyTree(pt, "s").getChildren()){
          ASSERT_LT(i, expectedKeys.size());
          EXPECT_EQ(expectedKeys[i++], c2.key);
        }
        EXPECT_EQ(expectedKeys.size(), i);
      }
    }
  catch(const std::exception & e)
    {
      FAIL() << "Unhandled exception: " << e.what();
    }
}

TEST(PTreeTestSuite, testRootNodeValues){
  sm::BoostPropertyTree pt;

  pt.setDouble("",0.01);

  EXPECT_EQ(pt.getDouble(""), 0.01);
}


TEST(PTreeTestSuite, testBoostPTreeUpdate)
{
  sm::BoostPropertyTree pt;
  pt.setDouble("d",0.1);
  pt.setDouble("d2",0.1);
  pt.setString("/s","hello");

  sm::BoostPropertyTree pt2;
  pt2.setDouble("d",0.2);
  pt2.setString("/s","hello2");

  ASSERT_EQ(pt.getDouble("d"), 0.1);
  ASSERT_EQ(pt.getDouble("d2"), 0.1);
  ASSERT_EQ(pt.getString("s"), std::string("hello"));
  ASSERT_NEAR(pt2.getDouble("d"), 0.2, 1e-16);
  ASSERT_EQ(pt2.getString("s"), std::string("hello2"));
  pt.update(pt2);
  ASSERT_EQ(pt.getDouble("d"), 0.2);
  ASSERT_EQ(pt.getDouble("d2"), 0.1);
  ASSERT_EQ(pt.getString("s"), std::string("hello2"));
}

TEST(PTreeTestSuite, testBoostPTreeUpdateOnlyException)
{
  sm::BoostPropertyTree pt;

  sm::BoostPropertyTree pt2;
  pt2.setDouble("d",0.2);

  ASSERT_THROW(pt.update(pt2, false), sm::PropertyTree::KeyNotFoundException);
}

TEST(PTreeTestSuite, testBoostPTreeLoadString)
{
  sm::BoostPropertyTree pt;
  pt.loadString("d=0.1,s=hello,a/d=0.1,b{u=3,v=4},c/b{u=3,v=4}");

  ASSERT_EQ(pt.getDouble("d"), 0.1);
  ASSERT_EQ(pt.getString("s"), std::string("hello"));
  ASSERT_EQ(pt.getDouble("a/d"), 0.1);
  ASSERT_EQ(pt.getInt("b/u"), 3);
  ASSERT_EQ(pt.getInt("b/v"), 4);
  ASSERT_EQ(pt.getInt("c/b/u"), 3);
  ASSERT_EQ(pt.getInt("c/b/v"), 4);
}


TEST(PTreeTestSuite, testFindFile)
{
  // Not finding should throw
  EXPECT_THROW(sm::findFile("BoostPropertyTreeImplementation.cpp", "RIDICULOUS_ENVIRONMENT_VARIABLE_THAT_CANNOT_EXIST"), std::runtime_error);
  EXPECT_THROW(sm::findFile("RidiculousFilenameThatCannotExist", "ROS_PACKAGE_PATH"), std::runtime_error);

  // Let's test whether we can find a file
  EXPECT_NO_THROW(sm::findFile(".", "PWD"));
}

TEST(PTreeTestSuite, testHumanReadable)
{
  sm::BoostPropertyTree::setHumanReadableInputOutput(true);
  sm::BoostPropertyTree wbpt;


  wbpt.setDouble("a",0.1);
  wbpt.setDouble("a/d",0.1);
  wbpt.setDouble("a/b",0.2);
  wbpt.setString("b","hello   "); // These spaces will get lost in a human readable XML context (see the expected value below)!
  wbpt.setString("b/h","hello");
  wbpt.setString("b/g","goodbye");
  const std::string xmlFile = "testHumanReadable.xml";
  wbpt.saveXml(xmlFile);

  std::string expectedXml[] ={
      "<?xml version=\"1.0\" encoding=\"utf-8\"?>",
      "<a>",
      "\t0.1",
      "\t<d>0.1</d>",
      "\t<b>0.2</b>",
      "</a>",
      "<b>",
      "\thello   ",
      "\t<h>hello</h>",
      "\t<g>goodbye</g>",
      "</b>"
  };

  try
  {
    std::ifstream xmlFileStream(xmlFile);
    int lineIndex = 0;
    for(std::string line; std::getline(xmlFileStream, line); lineIndex++){
      EXPECT_EQ(expectedXml[lineIndex], line);
    }
    xmlFileStream.close();

    sm::BoostPropertyTree pt;
    pt.loadXml(xmlFile);
    EXPECT_EQ(pt.getString("b"), std::string("hello"));
  }
  catch(const std::exception & e)
  {
    FAIL() << "Unhandled exception: " << e.what();
  }
}


TEST(PTreeTestSuite, testLoadXmlFromString)
{
  try{
      sm::BoostPropertyTree pt;

      pt.loadXmlFromString("<d>0.1</d>");

      EXPECT_NEAR(pt.getDouble("d"), 0.1, 1e-16);
  }
  catch(const std::exception & e)
  {
    FAIL() << "Unhandled exception: " << e.what();
  }
}
