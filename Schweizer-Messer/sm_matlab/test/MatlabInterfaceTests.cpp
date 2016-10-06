// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

#include <sm/random.hpp>

// Bring in the Matlab Interface
#include <sm/matlab/Engine.hpp>

TEST(MatlabInterfaceTestSuite, simpleOpenClose)
{
  sm::matlab::Engine engine;
  ASSERT_FALSE(engine.isInitialized());
  ASSERT_TRUE(engine.initialize());
  ASSERT_TRUE(engine.stop());
  ASSERT_FALSE(engine.isInitialized());
}

TEST(MatlabInterfaceTestSuite, simpleCommand)
{
  sm::matlab::Engine engine;
  engine.initialize();
  ASSERT_TRUE(engine.executeCommand("disp('test')")=="test");
}

TEST(MatlabInterfaceTestSuite, standardPut)
{
  sm::matlab::Engine engine;
  engine.initialize();
  
  double a = sm::random::rand();
  int b = static_cast<int>(sm::random::rand());
  uint64_t c = static_cast<uint64_t>(sm::random::rand());
  bool d = true;
  std::string e = "test";
  
  engine.put("a", a);
  engine.put("b", b);
  engine.put("c", c); 
  engine.put("d", d);
  engine.put("e", e);
}

TEST(MatlabInterfaceTestSuite, EigenPut)
{
  sm::matlab::Engine engine;
  engine.initialize();

  Eigen::Matrix2d A = Eigen::Matrix2d::Random();
  Eigen::Matrix3i B = Eigen::Matrix3i::Random();
  Eigen::Vector3d C = Eigen::Vector3d::Random();
  
  engine.putEigen("A", A);
  engine.putEigen("B", B);
  engine.putEigen("C", C); 
}

TEST(MatlabInterfaceTestSuite, standardTest)
{
  sm::matlab::Engine engine;
  engine.initialize();
  
  double a = sm::random::rand();
  int b = static_cast<int>(sm::random::rand());
  uint64_t c = static_cast<uint64_t>(sm::random::rand());
  bool d = true;
  std::string e = "test";
  
  engine.put("a", a);
  engine.put("b", b);
  engine.put("c", c); 
  engine.put("d", d);
  engine.put("e", e);
  
  engine.executeCommand("ab = a*b;");
  double abTest;
  ASSERT_TRUE(engine.get("ab", abTest));
  ASSERT_TRUE(abTest == a*static_cast<double>(b));
  
  std::string eTest;
  ASSERT_TRUE(engine.get("e", eTest));
  ASSERT_TRUE(e == eTest);
}

TEST(MatlabInterfaceTestSuite, EigenTest)
{
  sm::matlab::Engine engine;
  engine.initialize();
  
  Eigen::Matrix3d A = Eigen::Matrix3d::Random();
  Eigen::Matrix3i B = Eigen::Matrix3i::Random();
  Eigen::Matrix3d C = Eigen::Matrix3d::Random();
  Eigen::Vector2d D = Eigen::Vector2d::Random();
  
  engine.putEigen("A", A);
  engine.putEigen("B", B);
  engine.putEigen("C", C); 
  engine.putEigen("D", D);
  
  engine.executeCommand("AC = A+C;");
  Eigen::MatrixXd ACTest;
  Eigen::Matrix3d AC = A+C;
  ASSERT_TRUE(engine.getEigen("AC", ACTest));
  ASSERT_TRUE(ACTest == AC);

  Eigen::MatrixXd DTest;
  ASSERT_TRUE(engine.getEigen("D", DTest));
  ASSERT_TRUE(DTest == D);
}

TEST(MatlabInterfaceTestSuite, TestersTest)
{
  sm::matlab::Engine engine;
  engine.initialize();
  
  double a = sm::random::rand();
  std::string b = "test";
  Eigen::Matrix3d A = Eigen::Matrix3d::Random();
  
  engine.put("a", a);
  engine.put("b", b);
  engine.putEigen("A", A);
  
  ASSERT_TRUE(engine.exists("a"));
  ASSERT_TRUE(!engine.exists("x"));
  ASSERT_TRUE(engine.isScalar("a"));
  ASSERT_TRUE(!engine.isScalar("b"));
  ASSERT_TRUE(!engine.isEmpty("a"));
  ASSERT_TRUE(engine.isCharOrString("b"));
  ASSERT_TRUE(!engine.isCharOrString("a"));

  size_t rows, cols;
  engine.getDimensions("a", rows, cols);
  ASSERT_TRUE(rows==1 && cols==1);
  
  engine.getDimensions("A", rows, cols);
  ASSERT_TRUE(rows==3 && cols==3);
}



// /// Run all the tests that were declared with TEST()
// int main(int argc, char **argv){
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
