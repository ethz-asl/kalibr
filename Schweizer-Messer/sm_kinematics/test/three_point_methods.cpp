#include <gtest/gtest.h>
#include <sm/kinematics/three_point_methods.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/random.hpp>
#include <sm/kinematics/Transformation.hpp>

TEST(SmKinematicsTests, testThreePoint)
{
  typedef Eigen::MatrixXd matrix_type;
  typedef Eigen::Matrix3d matrix3;
  typedef Eigen::Matrix4d matrix4;
  typedef Eigen::Vector3d vector3;
  typedef Eigen::Vector4d vector4;
  using namespace sm;
  // Build a bunch of random points.
  const int N = 200;

  matrix_type p_a_ia;
  p_a_ia = Eigen::MatrixXd::Random(3,N) * 200.0;//matrix_type::Random(3,N);// * 10.f;
  
  sm::kinematics::Transformation T;
  T.setRandom();
  //matrix4 T_ba = T.T();
  matrix3 C_ba = T.C();
  vector3 rho_b_ab = T.t();
  
  //double scale = 2.0;
  double scale = 1.0;
  matrix_type p_b_ib(3,N);
	

  // Transform the points.
  std::vector<int> idxs(N);
  for(int c = 0; c < N; c++) {
    p_b_ib.col(c) = scale * C_ba * p_a_ia.col(c) + rho_b_ab;
    idxs[c] = c;
  }

  // Now, select random pairs of points and see if the three point algorithm can 
  // recover the transformation.
  matrix4 estT_ba;
  //double estScale;

  for(int NP = 3; NP < 10; NP++)
    {

      for(int i = 0; i < 200; i++) {
	std::random_shuffle(idxs.begin(), idxs.end());
	      
	matrix_type pa(3,NP);
	matrix_type pb(3,NP);

	// copy over the points.
	for(int c = 0; c < NP; c++) {
	  pa.col(c) = p_a_ia.col(idxs[c]);
	  pb.col(c) = p_b_ib.col(idxs[c]);
	}

	using namespace sm::kinematics;
	// run the 3-point algorithm
	estT_ba = threePointSvd(pb,pa);

	
	matrix3 estC_ba = estT_ba.topLeftCorner<3,3>();
	vector3 estrho_b_ab = estT_ba.topRightCorner<3,1>();

	// std::cout << "qreal:\n" << qreal << std::endl;
	// std::cout << "qest: \n" << qest << std::endl;
		
	// std::cout << "rhoreal:\n" << rho_b_ab << std::endl;
	// std::cout << "rhoest: \n" << estrho_b_ab << std::endl;

	//ASSERT_LT((qreal-qest).array().abs().sum(),1e-2);		
	ASSERT_LT((estC_ba - C_ba).array().abs().sum(),1e-2);
	ASSERT_LT((estrho_b_ab - rho_b_ab).array().abs().sum(),1e-2);
	
      }
    }

}



TEST(SmKinematicsTests, testQMethod)
{
  typedef Eigen::MatrixXd matrix_type;
  typedef Eigen::Matrix3d matrix3;
  typedef Eigen::Matrix4d matrix4;
  typedef Eigen::Vector3d vector3;
  typedef Eigen::Vector4d vector4;
  using namespace sm;
  // Build a bunch of random points.
  const int N = 200;

  matrix_type p_a;
  p_a = Eigen::MatrixXd::Random(3,N) * 200.0;//matrix_type::Random(3,N);// * 10.f;
  
  // Make them unit vectors
  for(int i = 0; i < p_a.cols(); i++)
    {
      p_a.col(i) = p_a.col(i) / p_a.col(i).norm();
    }

  matrix3 C_ba = sm::kinematics::axisAngle2R(sm::random::rand(), sm::random::rand(), sm::random::rand());
    
  //double scale = 2.0;
  //double scale = 1.0;
  matrix_type p_b = C_ba * p_a;
	


  std::vector<int> idxs(N);
  for(int c = 0; c < N; c++) {
    idxs[c] = c;
  }

  // Now, select random pairs of points and see if the three point algorithm can 
  // recover the transformation.
  Eigen::Matrix3d estC_ba;
  //double estScale;

  for(int NP = 2; NP < 10; NP++)
    {

      for(int i = 0; i < 200; i++) {
	std::random_shuffle(idxs.begin(), idxs.end());
	      
	matrix_type pa(3,NP);
	matrix_type pb(3,NP);

	// copy over the points.
	for(int c = 0; c < NP; c++) {
	  pa.col(c) = p_a.col(idxs[c]);
	  pb.col(c) = p_b.col(idxs[c]);
	}

	using namespace sm::kinematics;
	// run the 3-point algorithm
	estC_ba = qMethod(pb,pa);

	// std::cout << "qreal:\n" << qreal << std::endl;
	// std::cout << "qest: \n" << qest << std::endl;
		
	// std::cout << "rhoreal:\n" << rho_b_ab << std::endl;
	// std::cout << "rhoest: \n" << estrho_b_ab << std::endl;

	//ASSERT_LT((qreal-qest).array().abs().sum(),1e-2);		
	ASSERT_LT((estC_ba - C_ba).array().abs().sum(),1e-2);
    
	
      }
    }

}
