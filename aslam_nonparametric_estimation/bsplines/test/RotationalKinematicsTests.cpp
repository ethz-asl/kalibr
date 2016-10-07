// Bring in my package's API, which is what I'm testing
#include <sm/kinematics/RotationalKinematics.hpp>
#include <sm/kinematics/EulerAnglesZYX.hpp>
#include <sm/kinematics/EulerRodriguez.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/RotationVector.hpp>

// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from sm
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>

#include <Eigen/LU>

using namespace sm::kinematics;


// A helper class for estimating the S matrix using finite differences.
template<typename ROTATION_TYPE>
struct SMatrixFunctor
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef ROTATION_TYPE rotation_t;
  typedef Eigen::Vector3d input_t;
  typedef Eigen::Matrix3d jacobian_t;
  typedef Eigen::Vector3d value_t;
  typedef double scalar_t;

  SMatrixFunctor(const Eigen::Vector3d & pbar, const Eigen::Vector3d & v) :
    pbar_(pbar), v_(v)
  {
    Cbar_ = rotation.parametersToRotationMatrix(pbar);
  }

  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  Eigen::Vector3d operator()(const Eigen::Vector3d & dp)
  {
    Eigen::Matrix3d C = rotation.parametersToRotationMatrix(pbar_ + dp);
    
    Eigen::Vector3d Cv = C*v_;
    return Cv;
  }

  /// The nominal parameter value
  Eigen::Vector3d pbar_;
  Eigen::Vector3d v_;
  Eigen::Matrix3d Cbar_;
  rotation_t rotation;
};


// A helper class for estimating the S matrix using finite differences.
template<typename ROTATION_TYPE>
struct SMatrixFunctor2
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef ROTATION_TYPE rotation_t;
  typedef Eigen::Vector3d input_t;
  typedef Eigen::Matrix3d jacobian_t;
  typedef Eigen::Vector3d value_t;
  typedef double scalar_t;

  SMatrixFunctor2(const Eigen::Vector3d & pbar) :
    pbar_(pbar)
  {
    Cbar_ = rotation.parametersToRotationMatrix(pbar);
  }

  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  Eigen::Vector3d operator()(const Eigen::Vector3d & dp)
  {
    Eigen::Matrix3d C = rotation.parametersToRotationMatrix(pbar_ + dp);
    Eigen::Matrix3d wcross = C * Cbar_.transpose();
    
    Eigen::Vector3d result(wcross(1,2),-wcross(0,2),wcross(0,1));
    return result;
  }

  /// The nominal parameter value
  Eigen::Vector3d pbar_;
  Eigen::Vector3d v_;
  Eigen::Matrix3d Cbar_;
  rotation_t rotation;
};


// A helper class for estimating the S matrix using finite differences.
template<typename ROTATION_TYPE>
struct AngularVelocityFunctor
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef ROTATION_TYPE rotation_t;
  typedef Eigen::Matrix<double,6,1> input_t;
  typedef Eigen::Matrix<double,3,6> jacobian_t;
  typedef Eigen::Vector3d value_t;
  typedef double scalar_t;

  AngularVelocityFunctor(){}

  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  Eigen::Vector3d operator()(const Eigen::Matrix<double,6,1> & p)
  {
    return rotation.angularVelocityAndJacobian(p.head<3>(), p.tail<3>(), NULL);
  }

  rotation_t rotation;
};





template<typename ROTATION_TYPE>
class RotationalKinematicsTestHarness
{
public:
  typedef ROTATION_TYPE rotation_t;
  rotation_t rotation;

  void testParametersToRotationMatrix()
  {
    // Make sure that random parameters produce
    // a valid rotation matrix.
    srand(0);
    for(int i = 0; i < 20; i++)
      {
	
	Eigen::Vector3d p = Eigen::Vector3d::Random();
	Eigen::Matrix3d C = rotation.parametersToRotationMatrix(p);

	// Check that this is a rotation, not a reflection.
	double determinant = C.determinant();
	ASSERT_NEAR(determinant,1.0,1e-10);


	// Check that the matrix is orthonormal.
	Eigen::Matrix3d one = C.transpose() * C;
	sm::eigen::assertNear(one, Eigen::Matrix3d::Identity(), 1e-10, SM_SOURCE_FILE_POS);
      }
  }

  void testInvertibleParameterTransformation()
  {
    // Make sure that the parameter transformations are invertible.
    srand(1);
    for(int i = 0; i < 20; i++)
      {
	// We won't compare against these parameters because
	// often the C --> p transformation returns C to a particular range.
	Eigen::Vector3d p = Eigen::Vector3d::Random();
	if(i == 0)
	  {
	    // Test the boundary case
	    p = Eigen::Vector3d::Zero();
	  }

	Eigen::Matrix3d C1 = rotation.parametersToRotationMatrix(p);
	// Instead, we will use these parameters. Admittedly it makes for a
	// weaker unit test but I can't presently think of a generic way around.
	p = rotation.rotationMatrixToParameters(C1);
	Eigen::Matrix3d C2 = rotation.parametersToRotationMatrix(p);

	sm::eigen::assertNear(C1, C2, 1e-10, SM_SOURCE_FILE_POS);
      }
    
  }

  void testSMatrix()
  {
    srand(1);
    for(int i = 0; i < 20; i++)
      {
	// Choose a random parameter vector.
	Eigen::Vector3d pbar = Eigen::Vector3d::Random();
	if(i == 0)
	  {
	    // Test the boundary case
	    pbar = Eigen::Vector3d::Zero();
	  }

	Eigen::Vector3d v = Eigen::Vector3d::Random();
	Eigen::Matrix3d C1 = rotation.parametersToRotationMatrix(pbar);
	Eigen::Vector3d Cv = C1*v;
	// Make sure the vector is clamped to the valid range
	pbar = rotation.rotationMatrixToParameters(C1);
	
	// // Create a functor and the numerical diff struct
	SMatrixFunctor<rotation_t> functor(pbar,v);
	sm::eigen::NumericalDiff< SMatrixFunctor<rotation_t> > nd(functor, 1e-10);

	// Check that the estimated S matrix matches the actual S matrix
	Eigen::Matrix3d estCvCrossS = nd.estimateJacobian(Eigen::Vector3d::Zero());
	Eigen::Matrix3d CvCrossS = crossMx(Cv) * rotation.parametersToSMatrix(pbar);
	sm::eigen::assertNear(CvCrossS, estCvCrossS, 1e-5, SM_SOURCE_FILE_POS);

	SMatrixFunctor2<rotation_t> functor2(pbar);
	sm::eigen::NumericalDiff< SMatrixFunctor2<rotation_t> > nd2(functor2);

	Eigen::Matrix3d estS = nd2.estimateJacobian(Eigen::Vector3d::Zero());
	Eigen::Matrix3d S = rotation.parametersToSMatrix(pbar);

	sm::eigen::assertNear(S, estS, 1e-5, SM_SOURCE_FILE_POS);

      }   
  }

  void testAngularVelocity()
  {
    for(int i = 0; i < 10; i++)
      {
	Eigen::Matrix<double,6,1> p = Eigen::Matrix<double,6,1>::Random();
	AngularVelocityFunctor<rotation_t> avf;
	sm::eigen::NumericalDiff< AngularVelocityFunctor<rotation_t> > nd(avf);
	Eigen::Matrix<double,3,6> estJ = nd.estimateJacobian(p);
	Eigen::Matrix<double,3,6> J;
	rotation.angularVelocityAndJacobian(p.head<3>(),p.tail<3>(),&J);
	sm::eigen::assertNear(J, estJ, 1e-5, SM_SOURCE_FILE_POS);
      }
  }

  void testAll()
  {
    testParametersToRotationMatrix();
    testInvertibleParameterTransformation();
    testSMatrix();
    //testAngularVelocity();
  }
};


TEST(RotationalKinematicsTestSuite, testEulerAngles)
{
  RotationalKinematicsTestHarness<EulerAnglesZYX> eulerzyx;
  eulerzyx.testAll();
  eulerzyx.testAngularVelocity();
}


TEST(RotationalKinematicsTestSuite, testEulerRodriguez)
{
  RotationalKinematicsTestHarness<EulerRodriguez> rotationvector;
  rotationvector.testAll();
  rotationvector.testAngularVelocity();
  //rotationvector.testParametersToRotationMatrix();
  //rotationvector.testInvertibleParameterTransformation();
  //rotationvector.testParametersToRotationMatrix();
}


TEST(RotationalKinematicsTestSuite, testRotationVector)
 {
   RotationalKinematicsTestHarness<RotationVector> rotationvector;
   rotationvector.testAll();
 }

TEST(RotationalKinematicsTestSuite, testYpr)
 {
   RotationalKinematicsTestHarness<EulerAnglesYawPitchRoll> rotationvector;
   rotationvector.testAll();
 }
