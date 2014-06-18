// Bring in my package's API, which is what I'm testing
#include <bsplines/BSpline.hpp>

// Bring in gtest
#include <gtest/gtest.h>
#include <boost/cstdint.hpp>

// Helpful functions from schweizer_messer
#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>

#include <boost/tuple/tuple.hpp>


using namespace bsplines;

struct BSplineJacobianFunctor
{
  // Necessary for eigen fixed sized type member variables.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::VectorXd input_t;
  typedef Eigen::MatrixXd jacobian_t;
  typedef Eigen::VectorXd value_t;
  typedef double scalar_t;

  BSplineJacobianFunctor(BSpline bs, double t, int d) :
    bs_(bs), t_(t), d_(d)
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
    return bs_.evalD(t_,d_);
  }

  BSpline bs_;
  double t_;
  int d_;
};




// Check that the Jacobian calculation is correct.
TEST(SplineTestSuite, testBSplineJacobian)
{
  const int segments = 2;
  for(int order = 2; order < 10; order++)
    {
      BSpline bs(order);
      int nk = bs.numKnotsRequired(segments);
      std::vector<double> knots;
      for(int i = 0; i < nk; i++)
	{
	  knots.push_back(i);
	}
      
      for(int dim = 1; dim < 4; dim++)
	{
	  int nc = bs.numCoefficientsRequired(segments); 
	  Eigen::MatrixXd C = Eigen::MatrixXd::Random(dim,nc);
	  bs.setKnotsAndCoefficients(knots, C);

	  for(int derivative = 0; derivative < order; derivative++)
	    {
	      for(double t = bs.t_min(); t < bs.t_max(); t += 0.1)
		{ 
		  BSplineJacobianFunctor f(bs, t, derivative);
		  sm::eigen::NumericalDiff<BSplineJacobianFunctor> nd(f);
		  Eigen::MatrixXd estJ = nd.estimateJacobian(bs.localCoefficientVector(t));
		  Eigen::MatrixXd J;
		  Eigen::VectorXd v;
		  boost::tie(v,J) = bs.evalDAndJacobian(t, derivative);
		  
		  sm::eigen::assertNear(J, estJ, 1e-5, SM_SOURCE_FILE_POS);
		}
	    }
	}
      

    }

}



TEST(SplineTestSuite, testCoefficientMap)
{
  const int order = 4;
  const int segments = 10;
  const int dim = 5;
  BSpline bs(order);
  int nk = bs.numKnotsRequired(segments);
  int nc = bs.numCoefficientsRequired(segments); 
  
  std::vector<double> knots;
  for(int i = 0; i < nk; i++)
    {
      knots.push_back(i);
    }
     
  Eigen::MatrixXd C = Eigen::MatrixXd::Random(dim,nc);
  bs.setKnotsAndCoefficients(knots, C);

  const Eigen::MatrixXd & CC = bs.coefficients();
  for(int i = 0; i < bs.numVvCoefficients(); i++)
    {
      Eigen::Map<Eigen::VectorXd> m = bs.vvCoefficientVector(i);
      // Test pass by value...
      Eigen::Map<Eigen::Matrix<double, 5, 1> > m2 = bs.fixedSizeVvCoefficientVector<dim>(i);
      for(int r = 0; r < m.size(); ++r)
	{
	  ASSERT_TRUE( &m[r] == &CC(r,i) );
	  ASSERT_TRUE( &m[r] == &m2[r] );
	  m[r] = rand();
	  ASSERT_EQ( m[r], CC(r,i) );
	  ASSERT_EQ( m[r], m2[r] );
	  
	}
    }

}

TEST(SplineTestSuite, testGetBi)
{
	const int order = 4;
	const int segments = 10;
	const double startTime = 0;
	const double endTime = 5;
	const int numTimeSteps = 43;
	BSpline bs(order);
	bs.initConstantSpline(startTime, endTime, segments, Eigen::VectorXd::Zero(1));

	for(int i = 0; i <= numTimeSteps; i ++) {
		double t = startTime + (endTime - startTime) * ((double) i / numTimeSteps);
		Eigen::VectorXd localBiVector = bs.getLocalBiVector(t);
		SM_ASSERT_NEAR(std::runtime_error, localBiVector.sum(), 1.0, 1e-10, "the bis at a given time should always sum up to 1")
		Eigen::VectorXd biVector = bs.getBiVector(t);
		SM_ASSERT_NEAR(std::runtime_error, localBiVector.sum(), 1.0, 1e-10, "the bis at a given time should always sum up to 1")

		Eigen::VectorXd cumulativeBiVector = bs.getCumulativeBiVector(t);
		Eigen::VectorXd localCumulativeBiVector = bs.getLocalCumulativeBiVector(t);

		Eigen::VectorXi localCoefficientVectorIndices = bs.localCoefficientVectorIndices(t);
		int firstIndex = localCoefficientVectorIndices[0];
		SM_ASSERT_EQ(std::runtime_error, localCoefficientVectorIndices.size(), order, "localCoefficientVectorIndices has to have exactly " << order << " entries");

		for(int j = 0; j < order; j ++){
			SM_ASSERT_EQ(std::runtime_error, localCoefficientVectorIndices[j], firstIndex + j, "localCoefficientVectorIndices have to be successive");
		}

		for(int j = 0, n = biVector.size(); j < n; j++){
			if(j < firstIndex){
				SM_ASSERT_EQ(std::runtime_error, biVector[j], 0, "");
				SM_ASSERT_EQ(std::runtime_error, cumulativeBiVector[j], 1, "");
			} else if (j < firstIndex + order){
				SM_ASSERT_EQ(std::runtime_error, biVector[j], localBiVector[j - firstIndex], "localBiVector should be a slice of the biVector");
				SM_ASSERT_EQ(std::runtime_error, cumulativeBiVector[j], localCumulativeBiVector[j - firstIndex], "localCumulativeBiVector must be a slice of the cumulativeBiVector");
				SM_ASSERT_NEAR(std::runtime_error, cumulativeBiVector[j], localBiVector.segment(j - firstIndex, order - (j - firstIndex)).sum(), 1e-13, "cumulativeBiVector must be the sum of the localBiVector where it overlaps, but it is not at " << (j - firstIndex) << " (localBiVector=" << localBiVector << ")");
			} else {
				SM_ASSERT_EQ(std::runtime_error, biVector[j], 0, "at position " << j);
				SM_ASSERT_EQ(std::runtime_error, cumulativeBiVector[j], 0, "at position " << j);
			}
		}
	}
}

// TEST(SplineTestSuite, testBSplineCubic)
// {
//   double knots_d[] = {-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 };
//   double control_d[] = { 0.0,1.0, 2.0, 3.0 };
  
//   std::vector<double> knots;
//   knots.insert(knots.begin(),knots_d,knots_d + (sizeof(knots_d)/sizeof(double)));
  
//   std::vector<double> control;
//   control.insert(control.begin(),control_d,control_d + (sizeof(control_d)/sizeof(double)));

//   BSpline<double,4> s(knots,control);

//   for(double i = 1.0; i < 2.0; i += 0.1)
//     {
//       //std::cout << "s(" << i << ") = " << s.eval(i) << std::endl;
//       ASSERT_DOUBLE_EQ(s.eval(i),i);
//       //std::cout << "D1 s(" << i << ") = " << s.evalD(i,1) << std::endl;
//       //std::cout << "D2 s(" << i << ") = " << s.evalD(i,2) << std::endl;
//       //std::cout << "D3 s(" << i << ") = " << s.evalD(i,3) << std::endl;
//       //std::cout << "D4 s(" << i << ") = " << s.evalD(i,4) << std::endl;
//     }
  
//   // Check the bounds of evaluation. 
//   // Lower bound.
//   ASSERT_THROW(s.eval(0.999),Exception);
//   ASSERT_NO_THROW(s.eval(1.0));
//   // Upper bound
//   ASSERT_NO_THROW(s.eval(1.999));  
//   ASSERT_THROW(s.eval(2.0),Exception);
  
// }


// TEST(SplineTestSuite, testBSplineLinear)
// {
//   double knots_d[] = {-1.0, 0.0, 1.0, 2.0, 3.0, 4.0 };
//   double control_d[] = { 0.0,1.0, 2.0, 3.0 };
  
//   std::vector<double> knots;
//   knots.insert(knots.begin(),knots_d,knots_d + (sizeof(knots_d)/sizeof(double)));
  
//   std::vector<double> control;
//   control.insert(control.begin(),control_d,control_d + (sizeof(control_d)/sizeof(double)));

//   BSpline<double,2> s(knots,control);

//   for(double i = 0.0; i < 3.0; i += 0.1)
//     {
//       //std::cout << "s(" << i << ") = " << s.eval(i) << std::endl;
//       ASSERT_DOUBLE_EQ(s.eval(i),i);
//     }
  
//   // Check the bounds of evaluation. 
//   // Lower bound.
//   ASSERT_THROW(s.eval(-0.0001),Exception);
//   ASSERT_NO_THROW(s.eval(0.0));
//   // Upper bound
//   ASSERT_NO_THROW(s.eval(2.999));  
//   ASSERT_THROW(s.eval(3.0),Exception);

// }


// TEST(SplineTestSuite, testBSplineQuadratic)
// {
//   double knots_d[] = {-1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 };
//   double control_d[] = { 0.5,1.5, 2.5, 3.5 };
  
//   std::vector<double> knots;
//   knots.insert(knots.begin(),knots_d,knots_d + (sizeof(knots_d)/sizeof(double)));
  
//   std::vector<double> control;
//   control.insert(control.begin(),control_d,control_d + (sizeof(control_d)/sizeof(double)));

//   BSpline<double,3> s(knots,control);

//     for(double i = 1.0; i < 3.0; i += 0.1)
//     {
//       //std::cout << "s(" << i << ") = " << s.eval(i) << std::endl;
//       ASSERT_DOUBLE_EQ(s.eval(i),i);
//     }
  
//     // Check the bounds of evaluation. 
//   // Lower bound.
//   ASSERT_THROW(s.eval(0.999),Exception);
//   ASSERT_NO_THROW(s.eval(1.0));
//   // Upper bound
//   ASSERT_NO_THROW(s.eval(2.999));  
//   ASSERT_THROW(s.eval(3.0),Exception);
// }
