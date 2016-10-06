#include <gtest/gtest.h>
#include <sm/kinematics/transformations.hpp>
#include <sm/eigen/NumericalDiff.hpp>

using namespace sm::kinematics;
using namespace sm::eigen;

struct TransformationFunctor
{
  typedef Eigen::Matrix<double,6,1> input_t;
  typedef Eigen::Matrix<double,4,6> jacobian_t;
  typedef Eigen::Vector4d value_t;
  typedef double scalar_t;

  enum { XMinimalDimension = 6 };
  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  TransformationFunctor(Eigen::Matrix4d T, Eigen::Vector4d v) : T_(T), v_(v){}

  value_t operator()(input_t const & dt)
  {
    Eigen::Matrix4d T = toTEuler(dt) * T_;
    return T * v_; 
  }

  Eigen::Matrix4d T_;
  Eigen::Vector4d v_;
};


struct InverseTransformationFunctor
{
  typedef Eigen::Matrix<double,6,1> input_t;
  typedef Eigen::Matrix<double,4,6> jacobian_t;
  typedef Eigen::Vector4d value_t;
  typedef double scalar_t;

  enum { XMinimalDimension = 6 };
  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  InverseTransformationFunctor(Eigen::Matrix4d T, Eigen::Vector4d v) : T_(T), v_(v){}

  value_t operator()(input_t const & dt)
  {
    Eigen::Matrix4d T = toTEuler(dt) * T_;
    return inverseTransform(T) * v_; 
  }

  Eigen::Matrix4d T_;
  Eigen::Vector4d v_;
};



inline Eigen::Matrix4d randomTransformation()
{
  Eigen::Matrix<double,6,1> t = Eigen::Matrix<double,6,1>::Random();
  t.head<3>() *= 100.0;

  return toTEuler(t);
}

inline Eigen::Vector4d randomHomogeneousPoint()
{
  return Eigen::Vector4d::Random();
}

TEST(AsrlMathTests, testTransformationJacobians)
{
  srand(10);
  Eigen::Matrix4d T_ab = randomTransformation();
  Eigen::Vector4d v_b = randomHomogeneousPoint();
  NumericalDiff<TransformationFunctor> nd(TransformationFunctor(T_ab, v_b));

  TransformationFunctor::jacobian_t Jest = nd.estimateJacobian(TransformationFunctor::input_t::Zero());
  
  TransformationFunctor::jacobian_t J;
  Eigen::Vector4d v_a;
  transformationAndJacobian(T_ab, v_b, v_a, J);

  for(int r = 0; r < J.rows(); r++)
    {
      for(int c = 0; c < J.cols(); c++)
	{
	  ASSERT_NEAR(J(r,c),Jest(r,c),1e-4) << "Jacobian\n" << J << "\nand estimated Jacobian\n" << Jest << "\nwere not as close as expected\n";
	}
    }


}

TEST(AsrlMathTests, testTransformationJacobian)
{
  srand(11);
  Eigen::Matrix4d T_ab = randomTransformation();
  Eigen::Vector4d v_a = randomHomogeneousPoint();
  NumericalDiff<InverseTransformationFunctor> nd(InverseTransformationFunctor(T_ab, v_a));

  InverseTransformationFunctor::jacobian_t Jest = nd.estimateJacobian(InverseTransformationFunctor::input_t::Zero());
  
  InverseTransformationFunctor::jacobian_t J;
  Eigen::Vector4d v_b;
  inverseTransformationAndJacobian(T_ab, v_a, v_b, J);

  for(int r = 0; r < J.rows(); r++)
    {
      for(int c = 0; c < J.cols(); c++)
	{
	  ASSERT_NEAR(J(r,c),Jest(r,c),1e-4) << "Jacobian\n" << J << "\nand estimated Jacobian\n" << Jest << "\nwere not as close as expected\n";
	}
    }

}


TEST(AsrlMathTests, testTransformationSettersAndGetters)
{
  srand(4);

  Eigen::Matrix4d T = randomTransformation();
  Eigen::Matrix3d C = transform2C(T);
  Eigen::Vector3d p = transform2rho(T);
  Eigen::Vector4d hp = transform2rhoHomogeneous(T);  
  
  Eigen::Matrix4d bT = rt2Transform(C, p);
  ASSERT_TRUE( ((T-bT).array() == 0.0).all() ) << "The matrix T:\n" << T << "\nand the reconstructed matrix:\n" << bT << " are not equal"; 
  ASSERT_TRUE( ((T.topLeftCorner<3,3>()-C).array() == 0.0).all() ) << "The matrix T:\n" << T << "\nand the extracted rotation:\n" << C << " are not equal"; 
  ASSERT_TRUE( ((T.topRightCorner<3,1>()-p).array() == 0.0).all() ) << "The matrix T:\n" << T << "\nand the extracted translation:\n" << p << " are not equal"; 


  ASSERT_TRUE( ((T.topRightCorner<4,1>()-hp).array() == 0.0).all() ) << "The matrix T:\n" << T << "\nand the extracted homogeneous translation:\n" << hp << " are not equal"; 
  
  
}



TEST(AsrlMathTests, testTransformationToTEuler)
{
  srand(5);

  for(int i = 0; i < 10; i++)
    {
      Eigen::Matrix4d T = randomTransformation();

      Eigen::Matrix<double,6,1> t = fromTEuler(T);
	
      Eigen::Matrix4d eT = toTEuler(t);

      for(int r = 0; r < T.rows(); r++)
	{
	  for(int c = 0; c < T.cols(); c++)
	    {
	      ASSERT_NEAR(T(r,c),eT(r,c),1e-4) << "The transformation matrix\n" << T << "\nand the reconstructed matrix\n"
					       << eT << "\nare not equal";
	    }
	}
      
    }
}

TEST(AsrlMathTests, testBoxTimes)
{
  // We already know that inverseTransformationAndJacobian(...) works.
  // Let's use it to validate the boxtimes construction.
  srand(6);
  Eigen::Matrix4d T_ba = randomTransformation();
  Eigen::Vector4d v_b = Eigen::Vector4d::Random();
  Eigen::Vector4d v_a;
  Eigen::Matrix<double,4,6> B;
  Eigen::Matrix<double,4,6> bmB;

  inverseTransformationAndJacobian(T_ba, v_b, v_a, B); 

  bmB = -boxMinus(v_a) * boxTimes(inverseTransform(T_ba));

  for(int r = 0; r < bmB.rows(); r++)
    {
      for(int c = 0; c < bmB.cols(); c++)
	{
	  ASSERT_NEAR(bmB(r,c),B(r,c),1e-10) << "The jacobian\n" << B << "\nand alternately constructed jacobian\n"
					   << bmB << "\nare not equal";
	}
    }


}


TEST(AsrlMathTests, testToTEuler)
{
  Eigen::Matrix<double,6,1> t = Eigen::Matrix<double,6,1>::Random();

  Eigen::Matrix4d T1 = toTEuler(t);
  Eigen::Matrix4d T2 = toTEuler(t[0],t[1],t[2],t[3],t[4],t[5]);

  ASSERT_TRUE(T1 == T2) << "The two versions of toTEuler produce different transformation matrices:\n" << T1 << "\nand\n" << T2 << std::endl;
  
  
}
