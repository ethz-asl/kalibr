// Bring in my package's API, which is what I'm testing
#include <bsplines/BSplinePose.hpp>
#include <sparse_block_matrix/sparse_block_matrix.h>
#include <stdio.h>
// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from libsm
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>

#include <boost/tuple/tuple.hpp>
#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerRodriguez.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/EulerAnglesZYX.hpp>
#include <stdio.h>

using namespace bsplines;
using namespace sm::kinematics;

struct BSplineTransformationJacobianFunctor
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::VectorXd input_t;
  typedef Eigen::MatrixXd jacobian_t;
  typedef Eigen::VectorXd value_t;
  typedef double scalar_t;

  BSplineTransformationJacobianFunctor(BSplinePose bs, const Eigen::Vector4d & v, double t) :
    bs_(bs), t_(t), v_(v)
  {
    
  }

  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd & c)
  {
    bs_.setLocalCoefficientVector(t_,c);
    Eigen::Matrix4d T = bs_.transformation(t_);
    return T * v_;
  }

  BSplinePose bs_;
  double t_;
  Eigen::Vector4d v_;
};


struct BSplineInverseTransformationJacobianFunctor
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::VectorXd input_t;
  typedef Eigen::MatrixXd jacobian_t;
  typedef Eigen::VectorXd value_t;
  typedef double scalar_t;

  BSplineInverseTransformationJacobianFunctor(BSplinePose bs, const Eigen::Vector4d & v, double t) :
    bs_(bs), t_(t), v_(v)
  {
    
  }

  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd & c)
  {
    bs_.setLocalCoefficientVector(t_,c);
    Eigen::Matrix4d T = bs_.inverseTransformation(t_);
    return T * v_;
  }

  BSplinePose bs_;
  double t_;
  Eigen::Vector4d v_;
};





// Check that the Jacobian calculation is correct.
TEST(SplineTestSuite, testBSplineTransformationJacobian)
{
  boost::shared_ptr<RotationalKinematics> rvs[3];

  rvs[0].reset(new EulerAnglesZYX());
  rvs[1].reset(new RotationVector());
  rvs[2].reset(new EulerRodriguez());
  
  for(int r = 0; r < 3; r++)
  {
      for(int order = 2; order < 10; order++)
	{
	  // Create a two segment spline.
	  BSplinePose bs(order, rvs[r]);
	  bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(Eigen::VectorXd::Random(6)),
			    bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
	  bs.addPoseSegment(2.0,bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
      
	  // Create a random homogeneous vector.
	  Eigen::Vector4d v = Eigen::Vector4d::Random() * 10.0;

	  for(double t = bs.t_min(); t <= bs.t_max(); t+= 0.413)
	    {
	      BSplineTransformationJacobianFunctor f(bs, v, t);
	      sm::eigen::NumericalDiff<BSplineTransformationJacobianFunctor> nd(f);
	      Eigen::MatrixXd estJ = nd.estimateJacobian(bs.localCoefficientVector(t));
	      Eigen::MatrixXd JT;


	      Eigen::Matrix4d T = bs.transformationAndJacobian(t, &JT);

	      //std::cout << "Full jacobian:\n" << JT << std::endl;
	      //std::cout << "boxMinus(v):\n" << sm::kinematics::boxMinus(v) << std::endl;
	      Eigen::MatrixXd J = sm::kinematics::boxMinus(T*v) * JT;

	      sm::eigen::assertNear(J, estJ, 1e-6, SM_SOURCE_FILE_POS);

	      // Try again with the lumped function.
	      Eigen::Vector4d v_n = bs.transformVectorAndJacobian(t,v, &J);
	      sm::eigen::assertNear(v_n, T*v, 1e-6, SM_SOURCE_FILE_POS);
	      sm::eigen::assertNear(J, estJ, 1e-6, SM_SOURCE_FILE_POS);
	    }
	}
    }
      
}

#if 1
// Check that the Jacobian calculation is correct.
TEST(SplineTestSuite, testBSplineInverseTransformationJacobian)
{
  boost::shared_ptr<RotationalKinematics> rvs[3];

  rvs[0].reset(new EulerAnglesZYX());
  rvs[1].reset(new RotationVector());
  rvs[2].reset(new EulerRodriguez());
  
  for(int r = 0; r < 3; r++)
  {
      for(int order = 2; order < 10; order++)
	{
	  // Create a two segment spline.
	  BSplinePose bs(order, rvs[r]);
	  bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(Eigen::VectorXd::Random(6)),
			    bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
	  bs.addPoseSegment(2.0,bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
      
	  // Create a random homogeneous vector.
	  Eigen::Vector4d v = Eigen::Vector4d::Random() * 10.0;

	  for(double t = bs.t_min(); t <= bs.t_max(); t+= 0.413)
	    {
	      BSplineInverseTransformationJacobianFunctor f(bs, v, t);
	      sm::eigen::NumericalDiff<BSplineInverseTransformationJacobianFunctor> nd(f);
	      Eigen::MatrixXd estJ = nd.estimateJacobian(bs.localCoefficientVector(t));
	      Eigen::MatrixXd JT;
              Eigen::MatrixXd J;

	      Eigen::Matrix4d T = bs.inverseTransformationAndJacobian(t, &JT);

	      //std::cout << "Full jacobian:\n" << JT << std::endl;
	      //std::cout << "boxMinus(v):\n" << sm::kinematics::boxMinus(v) << std::endl;
	      J = sm::kinematics::boxMinus(T*v) * JT;
              //std::cout << "J(0,0): " << J(0,0) << std::endl;
	      sm::eigen::assertNear(J, estJ, 1e-6, SM_SOURCE_FILE_POS);

	      // Try again with the lumped function.
	      //Eigen::Vector4d v_n = bs.inverseTransformVectorAndJacobian(t,v, &J);
	      //sm::eigen::assertNear(v_n, T*v, 1e-6, SM_SOURCE_FILE_POS);
	      //sm::eigen::assertNear(J, estJ, 1e-6, SM_SOURCE_FILE_POS);
	    }
	}
    }
      
}

#endif


struct BSplineAccelerationJacobianFunctor
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::VectorXd input_t;
  typedef Eigen::MatrixXd jacobian_t;
  typedef Eigen::Vector3d value_t;
  typedef double scalar_t;

  BSplineAccelerationJacobianFunctor(BSplinePose bs, double t) :
    bs_(bs), t_(t)
  {
    
  }

  input_t update(const input_t & x, int c, double delta)
  {
    input_t xnew = x;
    xnew[c] += delta;
    return xnew;
  }

  Eigen::Vector3d operator()(const Eigen::VectorXd & c)
  {
    bs_.setLocalCoefficientVector(t_,c);
    Eigen::Vector3d a = bs_.linearAccelerationAndJacobian(t_,NULL,NULL);
    //std::cout << "a.size(): " << a.size() << std::endl;
    return a;
  }

  BSplinePose bs_;
  double t_;
};


TEST(SplineTestSuite, testBSplineAccelerationJacobian)
{
  boost::shared_ptr<RotationalKinematics> r(new RotationVector());
  
  for(int order = 2; order < 10; order++)
    {
      // Create a two segment spline.
      BSplinePose bs(order, r);
      bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(Eigen::VectorXd::Random(6)),
			bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
      bs.addPoseSegment(2.0,bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
     
      for(double t = bs.t_min(); t <= bs.t_max(); t+= 0.1)
	{

	  
	  Eigen::MatrixXd J;
	  bs.linearAccelerationAndJacobian(t,&J,NULL);

	  BSplineAccelerationJacobianFunctor f(bs,t);
	  sm::eigen::NumericalDiff<BSplineAccelerationJacobianFunctor> nd(f);
	  Eigen::MatrixXd estJ = nd.estimateJacobian(bs.localCoefficientVector(t));
	  

	  //std::cout << "J\n" << J << "\nestJ:\n" << estJ << std::endl;
	  sm::eigen::assertNear(J, estJ, 1e-6, SM_SOURCE_FILE_POS);
	  
	}
    }
}



TEST(SplineTestSuite, testBSplineCurveQuadraticIntegralSparse) {
    
    try {
    boost::shared_ptr<RotationalKinematics> rvs;
    
    rvs.reset(new RotationVector());
    

        const int length = 50;
        int numSegments = 20;
        double lambda = 0.5;
        
        
        for(int order = 2; order < 10; order++)
        {
            // Create a two segment spline.
            BSplinePose bs(order, rvs);
           // bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(Eigen::VectorXd::Random(6)),
           //                   bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
          //  bs.addPoseSegment(2.0,bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
            
            
            // random positive times
            Eigen::Matrix<double, length, 1> times = Eigen::VectorXd::Random(length) + Eigen::VectorXd::Ones(length);
            // sorted
            std::sort(&times.coeffRef(0), &times.coeffRef(0)+times.size());
            
            times[length-1] = ceil(times[length-1]);            
            
            Eigen::MatrixXd poses = Eigen::MatrixXd::Random(6,length);
            
            bs.initPoseSpline3(times,poses,numSegments,lambda);
            
            
            
            // random symmetric matrix:
            Eigen::MatrixXd W = Eigen::MatrixXd::Random(6,6);
            for (int i = 0; i < 6; i++) 
                for(int j = 0; j < i; j++)
                    W(j,i) = W(i,j);
            
            
            sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Q_sparse = bs.curveQuadraticIntegralSparse(W, 0);
        
            Eigen::MatrixXd Q_dense = bs.curveQuadraticIntegral(W, 0);   
            
            sm::eigen::assertNear(Q_dense, Q_sparse.toDense(), 1e-10, SM_SOURCE_FILE_POS);
        }
    }
    catch(const std::exception &e) {
        FAIL() << e.what();
    }
    
    
}



TEST(SplineTestSuite, testBSplineCurveQuadraticIntegralDiagSparse) {
    
    try {
        boost::shared_ptr<RotationalKinematics> rvs;
        
        rvs.reset(new RotationVector());
        const int length = 50;
        int numSegments = 20;
        double lambda = 0.5;
        
        for(int order = 2; order < 10; order++)
        {
            // Create a two segment spline.
            BSplinePose bs(order, rvs);
         //   bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(Eigen::VectorXd::Random(6)),
         //                     bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
         //   bs.addPoseSegment(2.0,bs.curveValueToTransformation(Eigen::VectorXd::Random(6)));
            
            // random positive times
            Eigen::Matrix<double, length, 1> times = Eigen::VectorXd::Random(length) + Eigen::VectorXd::Ones(length);
            // sorted
            std::sort(&times.coeffRef(0), &times.coeffRef(0)+times.size());
            
            times[length-1] = ceil(times[length-1]);            
            
            Eigen::MatrixXd poses = Eigen::MatrixXd::Random(6,length);
            
            bs.initPoseSpline3(times,poses,numSegments,lambda);
            
            
            // random symmetric matrix:
            Eigen::VectorXd W = Eigen::VectorXd::Random(6);
            
            
            sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Q_sparse = bs.curveQuadraticIntegralDiagSparse(W, 0);
            
            Eigen::MatrixXd Q_dense = bs.curveQuadraticIntegralDiag(W, 0);   
            
            sm::eigen::assertNear(Q_dense, Q_sparse.toDense(), 1e-10, SM_SOURCE_FILE_POS);
        }
    }
    catch(const std::exception &e) {
        FAIL() << e.what();
    }
    
    
}

#include <boost/progress.hpp>

TEST(SplineTestSuite, testInitSpline3Sparse) {
    
    try {
        boost::shared_ptr<RotationalKinematics> rvs;
        
        rvs.reset(new RotationVector());
        
        const bool benchmark = false;

        const int length = benchmark ? 1000 : 100;
        int numSegments = benchmark ? 200 : 20;
        double lambda = 0.1;
      //  int order = 2;
        for(int order = 2; order < 8; order++)
        {
            // Create a two segment spline.
            BSplinePose bs_sparse(order, rvs);
            BSplinePose bs_dense(order, rvs);
            
            // random positive times
            Eigen::Matrix<double, length, 1> times = Eigen::VectorXd::Random(length) + Eigen::VectorXd::Ones(length);
            // sorted
            std::sort(&times.coeffRef(0), &times.coeffRef(0)+times.size());
            
            times[length-1] = ceil(times[length-1]);

      //      times = times;

          //  std::cout << "Times:" << std::endl;
          //  std::cout << times << std::endl;
            
            Eigen::MatrixXd poses = Eigen::MatrixXd::Random(6,length);

            if(benchmark){
              {
                  boost::progress_timer timer;
                  std::cout << "Dense:" << std::endl;
                  bs_dense.initPoseSpline3(times,poses,numSegments,lambda);
              }
              {
                  boost::progress_timer timer;
                  std::cout << "Sparse:" << std::endl;
                  bs_sparse.initPoseSplineSparse(times,poses,numSegments,lambda);
              }
            }
            
            // diagonals
      //      for (int i = 0; i < Asparse.rows(); i++)
     //           std::cout << Asparse(i,i) << " : " << Adense(i,i) << std::endl;
            
            
            
            Eigen::MatrixXd m1 = bs_sparse.coefficients();
            Eigen::MatrixXd m2 = bs_dense.coefficients();

         /*   std::cout << "m1:" << std::endl;
            std::cout << m1 << std::endl;
            std::cout << "m2:" << std::endl;
            std::cout << m2 << std::endl;
*/
            sm::eigen::assertNear(m1, m2, 1e-8, SM_SOURCE_FILE_POS);
            
        }
    }
    catch(const std::exception &e) {
        FAIL() << e.what();
    }
    
    
}

