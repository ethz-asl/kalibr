#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/ErrorTerm.hpp>



using namespace aslam::backend;
using namespace sm::kinematics;




class HpErr : public aslam::backend::ErrorTermFs<4> 
{
public:
  aslam::backend::HomogeneousPoint & _p;
  typedef aslam::backend::ErrorTermFs<4> parent_t;
  HpErr(aslam::backend::HomogeneousPoint & p) : _p(p)
  {
    _p.setActive(true);
    _p.setBlockIndex(0);
    parent_t::setDesignVariables(&_p);
    parent_t::setInvR( inverse_covariance_t::Identity() );
  }
  virtual ~HpErr(){}

protected:
  /// \brief evaluate the error term
  virtual double evaluateErrorImplementation()
  {
      parent_t::setError(_p.toHomogeneous());
 
      return parent_t::evaluateChiSquaredError();
  }
 
  /// \brief evaluate the jacobian
  virtual void evaluateJacobiansImplementation(JacobianContainer & jacobians)
  {
    _p.evaluateJacobians(jacobians);
  }

  
  
};



struct HomogeneousExpressionNodeFunctor
{
  typedef Eigen::Vector4d value_t;
  typedef value_t::Scalar scalar_t;
  typedef Eigen::VectorXd input_t;
  typedef Eigen::MatrixXd jacobian_t;

  
  HomogeneousExpressionNodeFunctor(HomogeneousExpression dv) :
    _dv(dv) {}

  input_t update(const input_t & x, int c, scalar_t delta) { input_t xnew = x; xnew[c] += delta; return xnew; }

  HomogeneousExpression _dv;

  Eigen::VectorXd operator()(const Eigen::VectorXd & dr)
  {
    
    Eigen::Vector4d p = _dv.toHomogeneous();
    JacobianContainer J(4);
    _dv.evaluateJacobians(J);

    int offset = 0;
    for(size_t i = 0; i < J.numDesignVariables(); i++)
      {
	DesignVariable * d = J.designVariable(i);
	d->update(&dr[offset],d->minimalDimensions());
	offset += d->minimalDimensions();
      }

    p = _dv.toHomogeneous();
    
 
    for(size_t i = 0; i < J.numDesignVariables(); i++)
      {
	DesignVariable * d = J.designVariable(i);
	d->revertUpdate();
      }

    return p;
   
  }
};

//struct HomogeneousExpressionLogNodeFunctor
//{
//  typedef Eigen::Vector3d value_t;
//  typedef value_t::Scalar scalar_t;
//  typedef Eigen::Vector4d input_t;
//  typedef Eigen::MatrixXd jacobian_t;
//
//
//  HomogeneousExpressionLogNodeFunctor(HomogeneousExpression dv) :
//    _dv(dv) {}
//
//  input_t update(const input_t & x, int c, scalar_t delta) { input_t xnew = x; xnew[c] += delta; return xnew; }
//
//  HomogeneousExpression _dv;
//
//  Eigen::Vector3d operator()(const Eigen::Vector4d& dr)
//  {
//
//    Eigen::Vector3d p = sm::kinematics::qlog(_dv.toHomogeneous());
//    JacobianContainer J(4);
//    _dv.evaluateJacobians(J);
//
//    int offset = 0;
//    for(size_t i = 0; i < J.numDesignVariables(); i++)
//      {
//	DesignVariable * d = J.designVariable(i);
//	d->update(&dr[offset],d->minimalDimensions());
//	offset += d->minimalDimensions();
//      }
//
//    p = sm::kinematics::qlog(_dv.toHomogeneous());
//
//
//    for(size_t i = 0; i < J.numDesignVariables(); i++)
//      {
//	DesignVariable * d = J.designVariable(i);
//	d->revertUpdate();
//      }
//
//    return p;
//
//  }
//};



void testJacobian(HomogeneousExpression dv)
{
  HomogeneousExpressionNodeFunctor functor(dv);
  
  sm::eigen::NumericalDiff<HomogeneousExpressionNodeFunctor> numdiff(functor,1e-7);
  
  /// Discern the size of the jacobian container
//  Eigen::Vector4d p = dv.toHomogeneous();
  JacobianContainer Jc(4);
  dv.evaluateJacobians(Jc);
   
  Eigen::VectorXd dp(Jc.cols());
  dp.setZero();
  Eigen::MatrixXd Jest = numdiff.estimateJacobian(dp); 
  
  SCOPED_TRACE("");
  sm::eigen::assertNear(Jc.asSparseMatrix(), Jest, 1e-6, SM_SOURCE_FILE_POS, "Testing the quat Jacobian");
}

//void testLogJacobian(HomogeneousExpression dv)
//{
//  HomogeneousExpressionLogNodeFunctor functor(dv);
//
//  sm::eigen::NumericalDiff<HomogeneousExpressionNodeFunctor> numdiff(functor,1e-7);
//
//  /// Discern the size of the jacobian container
////  Eigen::Vector4d p = dv.toHomogeneous();
//  JacobianContainer Jc(3);
//  dv.evaluateJacobians(Jc);
//
//  Eigen::Vector3d dp(Jc.cols());
//  dp.setZero();
//  Eigen::MatrixXd Jest = numdiff.estimateJacobian(dp);
//
//  std::cout << "Jest is:" << std::endl << Jest << std::endl;
//  std::cout << "J real is:" << std::endl << Jest << std::endl;
//
//
//  SCOPED_TRACE("");
//  sm::eigen::assertNear(Jc.asSparseMatrix(), Jest, 1e-6, SM_SOURCE_FILE_POS, "Testing the quat Jacobian");
//}


//*
TEST(HomogeneousExpressionNodeTestSuites, testSimpleError)
{
  try
    {
      using namespace aslam::backend;
      

      using namespace sm::kinematics;
      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(0);
      HomogeneousExpression qr(&point);
      
      HpErr e(point);
      
      //SCOPED_TRACE("");
      //testJacobian(qr);

      JacobianContainer estJ( e.dimension() );
      e.evaluateJacobiansFiniteDifference(estJ);

      
      JacobianContainer J(e.dimension());      
      e.evaluateJacobians(J);


      SCOPED_TRACE("");
      sm::eigen::assertNear(J.asSparseMatrix(), estJ.asSparseMatrix(), 1e-6, SM_SOURCE_FILE_POS, "Checking the jacobian vs. finite differences");
      
    }
  catch(const std::exception & e)
    {
      FAIL() << e.what();
    }
}

//*/


// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testPoint)
{
  try 
    {
      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(0);
      HomogeneousExpression qr(&point);
      
      
      SCOPED_TRACE("");
      testJacobian(qr);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}



// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testEuclideanPoint)
{
  try 
    {
      EuclideanPoint point(Eigen::Vector3d::Random());
      point.setActive(true);
      point.setBlockIndex(0);
      HomogeneousExpression qr = point.toHomogeneousExpression();
      
      
      SCOPED_TRACE("");
      testJacobian(qr);
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}



// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testTransformedPoint)
{
  try 
    {
      RotationQuaternion quat(quatRandom());
      quat.setActive(true);
      quat.setBlockIndex(0);

      EuclideanPoint ep(Eigen::Vector3d::Random());
      ep.setActive(true);
      ep.setBlockIndex(1);
      RotationExpression rexp(&quat);
      EuclideanExpression eexp(&ep);
      TransformationBasic Tb(rexp, eexp);
      TransformationExpression T(&Tb);

      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(2);
      HomogeneousExpression p(&point);
      
      HomogeneousExpression Tp = T * p;
      
      sm::eigen::assertNear(Tp.toHomogeneous(), T.toTransformationMatrix() * p.toHomogeneous(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
      
      SCOPED_TRACE("");
      testJacobian(Tp);
      
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
}


// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testTransformedInverse)
{
  try 
    {
      RotationQuaternion quat(quatRandom());
      quat.setActive(true);
      quat.setBlockIndex(0);

      EuclideanPoint ep(Eigen::Vector3d::Random());
      ep.setActive(true);
      ep.setBlockIndex(1);
      RotationExpression rexp(&quat);
      EuclideanExpression eexp(&ep);
      TransformationBasic Tb(rexp, eexp);
      TransformationExpression T(&Tb);

      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(false);
      point.setBlockIndex(2);
      HomogeneousExpression p(&point);
      
      HomogeneousExpression Tp = T.inverse() * p;
      
      SCOPED_TRACE("");
      testJacobian(Tp);

      sm::eigen::assertNear(Tp.toHomogeneous(), T.toTransformationMatrix().inverse() * p.toHomogeneous(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 


// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testTransformationExpression1)
{
  try 
    {
      RotationQuaternion quat0(quatRandom());
      quat0.setActive(true);
      quat0.setBlockIndex(0);
      EuclideanPoint ep0(Eigen::Vector3d::Random());
      ep0.setActive(true);
      ep0.setBlockIndex(1);
      RotationExpression rexp0(&quat0);
      EuclideanExpression eexp0(&ep0);
      TransformationBasic Tb0(rexp0, eexp0);
      TransformationExpression T0(&Tb0);

      RotationQuaternion quat1(quatRandom());
      quat1.setActive(true);
      quat1.setBlockIndex(2);
      EuclideanPoint ep1(Eigen::Vector3d::Random());
      ep1.setActive(true);
      ep1.setBlockIndex(3);
      RotationExpression rexp1(&quat1);
      EuclideanExpression eexp1(&ep1);
      TransformationBasic Tb1(rexp1, eexp1);
      TransformationExpression T1(&Tb1);

  

      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(4);
      HomogeneousExpression p(&point);
      
      TransformationExpression T01 = T0 * T1;
      HomogeneousExpression T01p = T01 * p;
      
      SCOPED_TRACE("");
      testJacobian(T01p);

      sm::eigen::assertNear(T01p.toHomogeneous(), T0.toTransformationMatrix() * T1.toTransformationMatrix() * p.toHomogeneous(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 

// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testTransformationExpression2)
{
  try 
    {
      RotationQuaternion quat0(quatRandom());
      quat0.setActive(true);
      quat0.setBlockIndex(0);
      EuclideanPoint ep0(Eigen::Vector3d::Random());
      ep0.setActive(true);
      ep0.setBlockIndex(1);
      RotationExpression rexp0(&quat0);
      EuclideanExpression eexp0(&ep0);
      TransformationBasic Tb0(rexp0, eexp0);
      TransformationExpression T0(&Tb0);

      RotationQuaternion quat1(quatRandom());
      quat1.setActive(true);
      quat1.setBlockIndex(2);
      EuclideanPoint ep1(Eigen::Vector3d::Random());
      ep1.setActive(true);
      ep1.setBlockIndex(3);
      RotationExpression rexp1(&quat1);
      EuclideanExpression eexp1(&ep1);
      TransformationBasic Tb1(rexp1, eexp1);
      TransformationExpression T1(&Tb1);

      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(4);
      HomogeneousExpression p(&point);

      
      HomogeneousExpression T1p = T1 * p;
      HomogeneousExpression T01p = T0 * T1p;
      
      SCOPED_TRACE("");
      testJacobian(T01p);

      sm::eigen::assertNear(T01p.toHomogeneous(), T0.toTransformationMatrix() * T1.toTransformationMatrix() * p.toHomogeneous(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 


// Test that the jacobian matches the finite difference jacobian
TEST(HomogeneousExpressionNodeTestSuites, testTransformationExpression3)
{
  try 
    {
      RotationQuaternion quat0(quatRandom());
      quat0.setActive(true);
      quat0.setBlockIndex(0);
      EuclideanPoint ep0(Eigen::Vector3d::Random());
      ep0.setActive(true);
      ep0.setBlockIndex(1);
      RotationExpression rexp0(&quat0);
      EuclideanExpression eexp0(&ep0);
      TransformationBasic Tb0(rexp0, eexp0);
      TransformationExpression T0(&Tb0);

      RotationQuaternion quat1(quatRandom());
      quat1.setActive(true);
      quat1.setBlockIndex(2);
      EuclideanPoint ep1(Eigen::Vector3d::Random());
      ep1.setActive(true);
      ep1.setBlockIndex(3);
      RotationExpression rexp1(&quat1);
      EuclideanExpression eexp1(&ep1);
      TransformationBasic Tb1(rexp1, eexp1);
      TransformationExpression T1(&Tb1);

      HomogeneousPoint point(Eigen::Vector4d::Random());
      point.setActive(true);
      point.setBlockIndex(4);
      HomogeneousExpression p(&point);

      
      TransformationExpression T01 = T0 * T1;
      HomogeneousExpression T10p = T01.inverse() * p;
      
      SCOPED_TRACE("");
      testJacobian(T10p);

      sm::eigen::assertNear(T10p.toHomogeneous(), (T0.toTransformationMatrix() * T1.toTransformationMatrix()).inverse() * p.toHomogeneous(), 1e-14, SM_SOURCE_FILE_POS, "Testing the result is unchanged");
    }
  catch(std::exception const & e)
    {
      FAIL() << e.what();
    }
} 
