// Bring in my package's API, which is what I'm testing

#include <Eigen/Core>
// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Serialization
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Geometry>
#include <sm/eigen/serialization.hpp>



template<typename T, int A, int B, int C, int D>
void saveLoadMatrixTest()
{
  Eigen::Matrix<T,A,B> T1, T2;

  T1.resize(C,D);

  for(int r = 0; r < T1.rows(); r++)
    {
      for(int c = 0; c < T1.cols(); c++)
	{
	  T1(r,c) = (T)(((r-1)*T1.cols()) + c);
	}
    }

  {
    std::ofstream ofs("test.ba", std::ios::binary);
    boost::archive::binary_oarchive oa(ofs);
  
    oa << T1;
  
  }


  {
    std::ifstream ifs("test.ba", std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
  
    ia >> T2;
  }  



  for(int r = 0; r < T1.rows(); r++)
    {
      for(int c = 0; c < T1.cols(); c++)
	{
	  ASSERT_EQ(T1(r,c),T2(r,c)) << "Value at row " << r << " and col " << c << " is not exactly equal";
	}
    }

}


template<typename T, int A, int B, int C, int D>
void saveLoadXmlMatrixTest()
{
  Eigen::Matrix<T,A,B> T1, T2;

  T1.resize(C,D);

  for(int r = 0; r < T1.rows(); r++)
    {
      for(int c = 0; c < T1.cols(); c++)
	{
	  T1(r,c) = (T)(((r-1)*T1.cols()) + c);
	}
    }

  {
    std::ofstream ofs("test.xml", std::ios::binary);
    boost::archive::xml_oarchive oa(ofs);
  
    oa << ::boost::serialization::make_nvp("top",T1);
  
  }


  {
    std::ifstream ifs("test.xml", std::ios::binary);
    boost::archive::xml_iarchive ia(ifs);
  
    ia >> ::boost::serialization::make_nvp("top",T2);
  }  


  ASSERT_EQ(T1.rows(), T2.rows());
  ASSERT_EQ(T1.cols(), T2.cols());

  for(int r = 0; r < T1.rows(); r++)
    {
      for(int c = 0; c < T1.cols(); c++)
	{
	  ASSERT_EQ(T1(r,c),T2(r,c)) << "Value at row " << r << " and col " << c << " is not exactly equal";
	}
    }

}

TEST(EigenSerializationTestSuite, testMatrix)
{

  saveLoadMatrixTest<double,6,6,6,6>();
  saveLoadMatrixTest<float,6,6,6,6>();
  saveLoadMatrixTest<int,6,6,6,6>();
  saveLoadMatrixTest<unsigned,6,6,6,6>();
  saveLoadMatrixTest<char,6,6,6,6>();

  saveLoadMatrixTest<double,1,64,1,64>();  
  saveLoadMatrixTest<double,64,1,64,1>(); 
  saveLoadMatrixTest<float,1,64,1,64>();  
  saveLoadMatrixTest<float,64,1,64,1>();  

  saveLoadMatrixTest<double,Eigen::Dynamic,6,100,6>();
  saveLoadMatrixTest<double,6,Eigen::Dynamic,6,100>();
  saveLoadMatrixTest<double,Eigen::Dynamic,Eigen::Dynamic,100,100>();
  
}

TEST(EigenSerializationTestSuite, testMatrixXml)
{
  try{
	saveLoadXmlMatrixTest<double,6,6,6,6>();
	saveLoadXmlMatrixTest<float,6,6,6,6>();
	saveLoadXmlMatrixTest<int,6,6,6,6>();
	saveLoadXmlMatrixTest<unsigned,6,6,6,6>();
	saveLoadXmlMatrixTest<char,6,6,6,6>();

	saveLoadXmlMatrixTest<double,1,64,1,64>();  
	saveLoadXmlMatrixTest<double,64,1,64,1>(); 
	saveLoadXmlMatrixTest<float,1,64,1,64>();  
	saveLoadXmlMatrixTest<float,64,1,64,1>();  

	saveLoadXmlMatrixTest<double,Eigen::Dynamic,6,100,6>();
	saveLoadXmlMatrixTest<double,6,Eigen::Dynamic,6,100>();
	saveLoadXmlMatrixTest<double,Eigen::Dynamic,Eigen::Dynamic,100,100>();
  }
  catch(const std::exception & e)
	{
	  FAIL() << e.what();
	}
}


template<typename Transform_t>
void saveLoadTransformTest()
{
  
  Transform_t M1,M2;
  typedef typename Transform_t::Scalar Scalar;
  typename Transform_t::MatrixType & T1a = M1.matrix();

  for(int r = 0; r < T1a.rows(); r++)
    {
      for(int c = 0; c < T1a.cols(); c++)
	{
	  T1a(r,c) = (Scalar)(((r-1)*T1a.cols()) + c);
	}
    }

  {
    std::ofstream ofs("test.ba", std::ios::binary);
    boost::archive::binary_oarchive oa(ofs);
  
    oa << M1;
  
  }



  {
    std::ifstream ifs("test.ba", std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
  
    ia >> M2;
  }  

  typename Transform_t::MatrixType & T1 = M1.matrix();
  typename Transform_t::MatrixType & T2 = M2.matrix();

  for(int r = 0; r < T1.rows(); r++)
    {
      for(int c = 0; c < T1.cols(); c++)
	{
	  ASSERT_EQ(T1(r,c),T2(r,c)) << "Value at row " << r << " and col " << c << " is not exactly equal";
	}
    }

}

TEST(EigenSerializationTestSuite, testTransform)
{
  saveLoadTransformTest<Eigen::Affine2d>();
  saveLoadTransformTest<Eigen::Affine2f>();
  saveLoadTransformTest<Eigen::Affine3d>();
  saveLoadTransformTest<Eigen::Affine3f>();
  saveLoadTransformTest<Eigen::AffineCompact2d>();
  saveLoadTransformTest<Eigen::AffineCompact2f>();
  saveLoadTransformTest<Eigen::AffineCompact3d>();
  saveLoadTransformTest<Eigen::AffineCompact3f>();
  saveLoadTransformTest<Eigen::Isometry2f>();
  saveLoadTransformTest<Eigen::Isometry2d>();
  saveLoadTransformTest<Eigen::Isometry3f>();
  saveLoadTransformTest<Eigen::Isometry3d>();
  saveLoadTransformTest<Eigen::Projective2f>();
  saveLoadTransformTest<Eigen::Projective2d>();
  saveLoadTransformTest<Eigen::Projective3f>();
  saveLoadTransformTest<Eigen::Projective3d>();

  
}

// /// Run all the tests that were declared with TEST()
// int main(int argc, char **argv){
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
