#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>


template<int N>
struct NormalizeJacobianFunctor
{
  typedef Eigen::Matrix<double,N,1> value_t;
  typedef double scalar_t;
  typedef value_t input_t;
  typedef Eigen::Matrix<double,N,N> jacobian_t;

  input_t update(input_t pp, int dimension, double dpd)
  {
    pp[dimension] += dpd;
    return pp;
  }

  value_t operator()(const input_t & p)
      {
	return sm::kinematics::normalize(p);
      }
      
    };

TEST(HomogeneousCoordinatesTestSuite, testNormalize)
{
  
  for(int i = 0; i < 10; i++)
    {
      NormalizeJacobianFunctor<3> nj3;
      sm::eigen::NumericalDiff< NormalizeJacobianFunctor<3> > nd3(nj3);
      Eigen::Vector3d p3;
      p3.setRandom();
      Eigen::Matrix3d J3est = nd3.estimateJacobian(p3);
      Eigen::Matrix3d J3;
      sm::kinematics::normalizeAndJacobian(p3,J3);
      sm::eigen::assertNear(J3,J3est,1e-6,SM_SOURCE_FILE_POS, "Checking the normalize jacobian");
    }

  for(int i = 0; i < 10; i++)
    {
      NormalizeJacobianFunctor<4> nj4;
      sm::eigen::NumericalDiff< NormalizeJacobianFunctor<4> > nd4(nj4);
      Eigen::Vector4d p4;
      p4.setRandom();
      Eigen::Matrix4d J4est = nd4.estimateJacobian(p4);
      Eigen::Matrix4d J4;
      sm::kinematics::normalizeAndJacobian(p4,J4);
      sm::eigen::assertNear(J4,J4est,1e-6,SM_SOURCE_FILE_POS, "Checking the normalize jacobian");
    }
	  
 
}

TEST(HomogeneousCoordinatesTestSuite, testPlus)
{
  using namespace sm::kinematics;
  for(int i = 0; i < 10; ++i)
    {
      Eigen::Vector4d ph1, ph2;
      ph1.setRandom();
      ph2.setRandom();
      ph1.array() -= 0.5;
      ph2.array() -= 0.5;

      Eigen::Vector4d ph12 = toHomogeneousPlus(ph1) * ph2;

      Eigen::Vector3d p1, p2, p12;
      p1 = fromHomogeneous(ph1);
      p2 = fromHomogeneous(ph2);
      p12 = fromHomogeneous(ph12);

      sm::eigen::assertNear(p12,p1 + p2,1e-6,SM_SOURCE_FILE_POS, "the addition of homogeneous points by matrix multiplication");
    }
}


TEST(HomogeneousCoordinatesTestSuite, testMinus)
{
  using namespace sm::kinematics;
  for(int i = 0; i < 10; ++i)
    {
      Eigen::Vector4d ph1, ph2;
      ph1.setRandom();
      ph2.setRandom();
      ph1.array() -= 0.5;
      ph2.array() -= 0.5;

      Eigen::Vector4d ph12 = toHomogeneousMinus(ph1) * ph2;

      Eigen::Vector3d p1, p2, p12;
      p1 = fromHomogeneous(ph1);
      p2 = fromHomogeneous(ph2);
      p12 = fromHomogeneous(ph12);

      sm::eigen::assertNear(p12,p2 - p1,1e-6,SM_SOURCE_FILE_POS, "the subtraction of homogeneous points by matrix multiplication");
    }
}

