#ifndef SM_POINT_TEST_HARNESS_HPP
#define SM_POINT_TEST_HARNESS_HPP

#include <sm/eigen/gtest.hpp>
#include <boost/bind.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/boost/serialization.hpp>

namespace sm {
  
  template<typename POINT_T>
  class PointTestHarness
  {
  public:
    typedef POINT_T point_t;
    double _threshold;
    PointTestHarness(double nearnessThreshold = 1e-10) : _threshold(nearnessThreshold) {}
    

    void testAdd(double threshold = -1)
    {
	  SCOPED_TRACE(__FUNCTION__);
      if(threshold < 0) 
	threshold = _threshold;

      Eigen::Vector3d p1;
      Eigen::Vector3d p2;
      Eigen::Vector3d answer;
      for(int i = 0; i < 20; i++)
	{
	  p1.setRandom();
	  p2.setRandom();
	  answer = p1+p2;
	 
	  point_t P1(p1);
	  point_t P2(p2);
	  
	  point_t Answer = P1 + P2;
	  
	  sm::eigen::assertNear(Answer.toEuclidean(), answer, threshold, SM_SOURCE_FILE_POS);
	}
    }

    void testSubtract(double threshold = -1)
    {
	  SCOPED_TRACE(__FUNCTION__);
      if(threshold < 0) 
	threshold = _threshold;

      Eigen::Vector3d p1;
      Eigen::Vector3d p2;
      Eigen::Vector3d answer;
      for(int i = 0; i < 20; i++)
	{
	  p1.setRandom();
	  p2.setRandom();
	  answer = p1-p2;
	 
	  point_t P1(p1);
	  point_t P2(p2);
	  
	  point_t Answer = P1 - P2;
	  
	  sm::eigen::assertNear(Answer.toEuclidean(), answer, threshold, SM_SOURCE_FILE_POS);
	}
    }

    void testAssign3(double threshold = -1)
    {
	  SCOPED_TRACE(__FUNCTION__);
      if(threshold < 0) 
	threshold = _threshold;

      Eigen::Vector3d p1;
      for(int i = 0; i < 20; i++)
	{
	  p1.setRandom();
	 
	  point_t P1;

	  P1 = p1;

	  sm::eigen::assertNear(P1.toEuclidean(), p1, threshold, SM_SOURCE_FILE_POS);
	}
 
    }

    void testAssign4(double threshold = -1.0)
    {
	  SCOPED_TRACE(__FUNCTION__);
      if(threshold < 0) 
	threshold = _threshold;

      Eigen::Vector4d p1;
      for(int i = 0; i < 20; i++)
	{
	  p1.setRandom();
	 
	  point_t P1;

	  P1 = p1;

	  Eigen::Vector4d p1a = p1/p1.norm();
	  Eigen::Vector4d P1a = P1.toHomogeneous() / P1.toHomogeneous().norm();

	  sm::eigen::assertNear(p1a, P1a, threshold, SM_SOURCE_FILE_POS);
	}
 
    }


    struct EuclideanJacobianFunctor
    {
      typedef Eigen::Vector3d value_t;
      typedef double scalar_t;
      typedef Eigen::Vector3d input_t;
      typedef Eigen::Matrix3d jacobian_t;

      point_t p;
      EuclideanJacobianFunctor(point_t p) : p(p) {}

      Eigen::Vector3d update(Eigen::Vector3d pp, int dimension, double dpd)
      {
	pp[dimension] += dpd;
	return pp;
      }

      value_t operator()(const Eigen::Vector3d & dp)
      {
	point_t j = p;
	j.oplus(dp);
	return j.toEuclidean();
      }
      
    };
    
    void testToEuclideanJacobian(double threshold = -1)
    {
	  SCOPED_TRACE(__FUNCTION__);
      if(threshold < 0) 
	threshold = _threshold;

      for(int i = 0; i < 10; i++)
	{
	  point_t P;
	  P.setRandom();
	  EuclideanJacobianFunctor functor(P);
	  sm::eigen::NumericalDiff<EuclideanJacobianFunctor> nd(functor);
	  Eigen::Matrix3d Jest = nd.estimateJacobian(Eigen::Vector3d::Zero());
	  Eigen::Matrix3d J;
	  P.toEuclideanAndJacobian(J);
	  
	  sm::eigen::assertNear(J,Jest, threshold, SM_SOURCE_FILE_POS);	  
	}

    }


    struct HomogeneousJacobianFunctor
    {
	  
      typedef Eigen::Vector4d value_t;
      typedef double scalar_t;
      typedef Eigen::Vector3d input_t;
      typedef Eigen::Matrix<double,4,3> jacobian_t;

      point_t p;
      HomogeneousJacobianFunctor(point_t p) : p(p) {}

      Eigen::Vector3d update(Eigen::Vector3d pp, int dimension, double dpd)
      {
	pp[dimension] += dpd;
	return pp;
      }

      value_t operator()(const Eigen::Vector3d & dp)
      {
	point_t j = p;
	j.oplus(dp);
	return j.toHomogeneous();
      }
      
    };
    
    void testToHomogeneousJacobian(double threshold = -1)
    {
	  SCOPED_TRACE(__FUNCTION__);
      if(threshold < 0) 
	threshold = _threshold;

      for(int i = 0; i < 10; i++)
	{
	  point_t P;
	  P.setRandom();
	  HomogeneousJacobianFunctor functor(P);
	  sm::eigen::NumericalDiff<HomogeneousJacobianFunctor> nd(functor);
	  Eigen::Matrix<double,4,3> Jest = nd.estimateJacobian(Eigen::Vector3d::Zero());
	  Eigen::Matrix<double,4,3> J;
	  P.toHomogeneousAndJacobian(J);
	  
	  sm::eigen::assertNear(J,Jest, threshold, SM_SOURCE_FILE_POS);	  
	}

    }


	void testSerialization()
	{
	  SCOPED_TRACE(__FUNCTION__);

	  point_t p1;
	  p1.setRandom();
	  
	  sm::boost_serialization::save(p1, "test.ba");

	  ASSERT_TRUE(p1.isBinaryEqual(p1));
	  
	  point_t p2;

	  ASSERT_FALSE(p1.isBinaryEqual(p2));
	  ASSERT_FALSE(p2.isBinaryEqual(p1));

	  sm::boost_serialization::load(p2, "test.ba");

	  ASSERT_TRUE(p1.isBinaryEqual(p2));
	  ASSERT_TRUE(p2.isBinaryEqual(p1));
	  
	  p1.setRandom();

	  ASSERT_FALSE(p1.isBinaryEqual(p2));
	  ASSERT_FALSE(p2.isBinaryEqual(p1));

	  sm::boost_serialization::save_xml(p1, "Point", "test.xml");

	  sm::boost_serialization::load_xml(p2, "Point", "test.xml");

	  // Too strict.
	  //ASSERT_TRUE(p1.isBinaryEqual(p2));
	  //ASSERT_TRUE(p2.isBinaryEqual(p1));

	}

    void testAll()
    {
      testAdd();
      testSubtract();
      testAssign3();
      testAssign4();
      testToEuclideanJacobian();
      testToHomogeneousJacobian();
	  testSerialization();
    }
    
  };
  
} // namespace sm


#endif /* SM_POINT_TEST_HARNESS_HPP */
