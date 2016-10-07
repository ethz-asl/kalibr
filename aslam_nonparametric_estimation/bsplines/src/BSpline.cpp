#include <sparse_block_matrix/sparse_block_matrix.h>
#include <sparse_block_matrix/linear_solver_cholmod.h>
#include <bsplines/BSpline.hpp>
#include <sm/assert_macros.hpp>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
//#include <asrl/string_routines.hpp>
// boost::tie()
#include <boost/tuple/tuple.hpp>
#include <Eigen/SVD> 

namespace bsplines {
    
    BSpline::BSpline(int splineOrder)
      : splineOrder_(splineOrder)
    {
      SM_ASSERT_GE(Exception, splineOrder_, 2, "The B-spline order must be greater than or equal to 2");
    }

    BSpline::~BSpline()
    {

    }
      
    int BSpline::splineOrder() const
    {
      return splineOrder_;
    }

    int BSpline::polynomialDegree() const
    {
      return splineOrder_ - 1;
    }

    void BSpline::setKnotsAndCoefficients(const std::vector<double> & knots, const Eigen::MatrixXd & coefficients)
    {
      //std::cout << "setting " << knots.size() << " knots\n";
      // This will throw an exception if it is an invalid knot sequence.
      verifyKnotSequence(knots);

      // Check if the number of coefficients matches the number of knots.
      SM_ASSERT_EQ(Exception, 
		     numCoefficientsRequired(numValidTimeSegments(knots.size())),
		     coefficients.cols(),
		     "A B-spline of order " << splineOrder_ << " requires " << numCoefficientsRequired(numValidTimeSegments(knots.size()))
		     << " coefficients for the " << numValidTimeSegments(knots.size()) 
		     << " time segments defined by " << knots.size() << " knots");  
      
      //std::cout << "Setting coefficients: " << coefficients << std::endl;

      knots_ = knots;
      coefficients_ = coefficients;

      initializeBasisMatrices();
    }

    void BSpline::initializeBasisMatrices()
    {
      basisMatrices_.resize(numValidTimeSegments());

      for(unsigned i = 0; i < basisMatrices_.size(); i++)
	{
	  basisMatrices_[i] = M(splineOrder_,i + splineOrder_ - 1);
//	  std::cout << "M[" << i << "]:\n" << basisMatrices_[i] << std::endl;
	}
    }


    Eigen::MatrixXd BSpline::M(int k, int i)
    {
      SM_ASSERT_GE_DBG(Exception, k, 1, "The parameter k must be greater than or equal to 1");
      SM_ASSERT_GE_DBG(Exception, i, 0, "The parameter i must be greater than or equal to 0");
      SM_ASSERT_LT_DBG(Exception, i, (int)knots_.size(), "The parameter i must be less than the number of time segments");
      if(k == 1)
	{
	  // The base-case for recursion.
	  Eigen::MatrixXd M(1,1);
	  M(0,0) = 1;
	  return M;
	}
      else
	{
	  Eigen::MatrixXd M_km1 = M(k-1,i);
	  // The recursive equation for M
	  // M_k = [ M_km1 ] A  + [  0^T  ] B
	  //       [  0^T  ]      [ M_km1 ]
	  //        -------        -------
	  //         =: M1          =: M2
	  //
	  //     = M1 A + M2 B
	  Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(M_km1.rows() + 1, M_km1.cols());
	  Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(M_km1.rows() + 1, M_km1.cols());

	  M1.topRightCorner(M_km1.rows(),M_km1.cols()) = M_km1;
	  M2.bottomRightCorner(M_km1.rows(),M_km1.cols()) = M_km1;

	  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(k-1, k);
	  for(int idx = 0; idx < A.rows(); idx++)
	    {
	      int j = i - k + 2 + idx;
	      double d0 = d_0(k, i, j);
	      A(idx, idx  ) = 1.0 - d0;
	      A(idx, idx+1) = d0;
	    }

	  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(k-1, k);
	  for(int idx = 0; idx < B.rows(); idx++)
	    {
	      int j = i - k + 2 + idx;
	      double d1 = d_1(k, i, j);
	      B(idx, idx  ) = -d1;
	      B(idx, idx+1) = d1;
	    }
	  
	  
	  Eigen::MatrixXd M_k;

	  return M_k = M1 * A + M2 * B;
	}
    }

    double BSpline::d_0(int k, int i, int j)
    {
      SM_ASSERT_GE_LT_DBG(Exception,j+k-1,0,(int)knots_.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
      SM_ASSERT_GE_LT_DBG(Exception,j,0,(int)knots_.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
      SM_ASSERT_GE_LT_DBG(Exception,i,0,(int)knots_.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
      double denom = knots_[j+k-1] - knots_[j];
      if(denom <= 0.0)
	return 0.0;

      double numerator = knots_[i] - knots_[j];

      return numerator/denom;
    }

    double BSpline::d_1(int k, int i, int j)
    {
      SM_ASSERT_GE_LT_DBG(Exception,j+k-1,0,(int)knots_.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
      SM_ASSERT_GE_LT_DBG(Exception,i+1,0,(int)knots_.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
      SM_ASSERT_GE_LT_DBG(Exception,i,0,(int)knots_.size(), "Index out of range with k=" << k << ", i=" << i << ", and j=" << j);
      double denom = knots_[j+k-1] - knots_[j];
      if(denom <= 0.0)
	return 0.0;

      double numerator = knots_[i+1] - knots_[i];

      return numerator/denom;
    }



    void BSpline::setKnotVectorAndCoefficients(const Eigen::VectorXd & knots, const Eigen::MatrixXd & coefficients)
    {
      //std::cout << "setting knots of size " << knots.size() << std::endl;//": " << knots.transpose() << std::endl;
      std::vector<double> k(knots.size());
      for(unsigned i = 0; i < k.size(); i++)
	k[i] = knots(i);

      setKnotsAndCoefficients(k, coefficients);
    }

    const std::vector<double> BSpline::knots() const
    {
      return knots_;
    }
    
    Eigen::VectorXd BSpline::knotVector() const
    {
      Eigen::VectorXd k(knots_.size());
      for(unsigned i = 0; i < knots_.size(); i++)
	k(i) = knots_[i];

      return k;
    }

    const Eigen::MatrixXd & BSpline::coefficients() const
    {
      return coefficients_;
    }
    

    void BSpline::verifyKnotSequence(const std::vector<double> & knots) 
    {
      SM_ASSERT_GE(Exception, (int)knots.size(), minimumKnotsRequired(), 
		     "The sequence does not contain enough knots to define an active time sequence "
		     << "for a B-spline of order " << splineOrder_ << ". At least " << minimumKnotsRequired() 
		     << " knots are required");
      
      for(unsigned i = 1; i < knots_.size(); i++)
	{
	  SM_ASSERT_LE(Exception, knots[i-1], knots[i],
			 "The knot sequence must be nondecreasing. Knot " << i
			 << " was not greater than or equal to knot " << (i-1));
	}
    }
    
    int BSpline::numValidTimeSegments(int numKnots) const
    {
      int nv = numKnots - 2*splineOrder_ + 1;
      return std::max(nv,0);
    }

    int BSpline::numValidTimeSegments() const
    {
      return numValidTimeSegments(knots_.size());
    }
    
    int BSpline::minimumKnotsRequired() const
    {
      return numKnotsRequired(1);
    }

    int BSpline::numCoefficientsRequired(int numTimeSegments) const
    {
      return numTimeSegments + splineOrder_ - 1;
    }   

    int BSpline::numKnotsRequired(int numTimeSegments) const
    {
      return numCoefficientsRequired(numTimeSegments) + splineOrder_;
    }   


    double BSpline::t_min() const
    {
      SM_ASSERT_GE(Exception, (int)knots_.size(), minimumKnotsRequired(), "The B-spline is not well initialized");
      return knots_[splineOrder_ - 1];
    }

    double BSpline::t_max() const
    {
      SM_ASSERT_GE(Exception, (int)knots_.size(), minimumKnotsRequired(), "The B-spline is not well initialized");
      return knots_[knots_.size() - splineOrder_];
    }

    std::pair<double,int> BSpline::computeTIndex(double t) const
    {
      SM_ASSERT_GE(Exception, t, t_min(), "The time is out of range by " << (t - t_min()));
        
        //// HACK - avoids numerical problems on initialisation
        if ( fabs(t_max() - t) < 1e-10 )
            t = t_max();
        //// \HACK
        
      SM_ASSERT_LE(Exception, t, t_max(), "The time is out of range by " << (t_max() - t));
      std::vector<double>::const_iterator i;
      if(t == t_max())
	{
	  // This is a special case to allow us to evaluate the spline at the boundary of the
	  // interval. This is not stricly correct but it will be useful when we start doing
	  // estimation and defining knots at our measurement times.
	  i = knots_.end() - splineOrder_;
	}
      else
	{
	  i = std::upper_bound(knots_.begin(), knots_.end(), t);
	}
      SM_ASSERT_TRUE_DBG(Exception, i != knots_.end(), "Something very bad has happened in computeTIndex(" << t << ")");
      
      // Returns the index of the knot segment this time lies on and the width of this knot segment.
      return std::make_pair(*i - *(i-1),(i - knots_.begin()) - 1);

    }

    std::pair<double,int> BSpline::computeUAndTIndex(double t) const 
    {
      std::pair<double,int> ui = computeTIndex(t);
      
      int index = ui.second;
      double denom = ui.first;

      if(denom <= 0.0)
	{
	  // The case of duplicate knots.
	  //std::cout << "Duplicate knots\n";
	  return std::make_pair(0, index);
	}
      else
	{

    //	  std::cout << "u:" << t << ", " << knots_[index] << ", " << denom << " idx:" << index;

	  double u = (t - knots_[index])/denom;
	  return std::make_pair(u, index);
	}
    }

    int dmul(int i, int derivativeOrder)
    {
      if(derivativeOrder == 0)
	return 1;
      else if(derivativeOrder == 1)
	return i;
      else
	return i * dmul(i-1,derivativeOrder-1) ;
    }


    Eigen::VectorXd BSpline::computeU(double uval, int segmentIndex, int derivativeOrder) const
    {
      Eigen::VectorXd u = Eigen::VectorXd::Zero(splineOrder_);
      double delta_t = knots_[segmentIndex+1] - knots_[segmentIndex]; 
      double multiplier = 0.0;
      if(delta_t > 0.0)
	multiplier = 1.0/pow(delta_t, derivativeOrder);

      double uu = 1.0;
      for(int i = derivativeOrder; i < splineOrder_; i++)
	{
	  u(i) = multiplier * uu * dmul(i,derivativeOrder) ; 
	  uu = uu * uval;
	}
  //    std::cout << "u:" << std::endl;
  //    std::cout << u << std::endl;

      return u;
    }

    Eigen::VectorXd BSpline::eval(double t) const
    {
      return evalD(t,0);
    }
    
    const Eigen::MatrixXd & BSpline::basisMatrixFromKnotIndex(int knotIndex) const
    {
      return basisMatrices_[basisMatrixIndexFromStartingKnotIndex(knotIndex)];
    }


    Eigen::VectorXd BSpline::evalD(double t, int derivativeOrder) const
    {
      SM_ASSERT_GE(Exception, derivativeOrder, 0, "To integrate, use the integral function");
      // Returns the normalized u value and the lower-bound time index.
      std::pair<double,int> ui = computeUAndTIndex(t);
      Eigen::VectorXd u = computeU(ui.first, ui.second, derivativeOrder);
      
      int bidx = ui.second - splineOrder_ + 1;

      // Evaluate the spline (or derivative) in matrix form.
      //
      // [c_0 c_1 c_2 c_3] * B^T * u
      // spline coefficients      

      Eigen::VectorXd rv = coefficients_.block(0,bidx,coefficients_.rows(),splineOrder_) * basisMatrices_[bidx].transpose() * u;

      return rv;

    }

    Eigen::VectorXd BSpline::evalDAndJacobian(double t, int derivativeOrder, Eigen::MatrixXd * Jacobian, Eigen::VectorXi * coefficientIndices) const
    {
      SM_ASSERT_GE(Exception, derivativeOrder, 0, "To integrate, use the integral function");
      // Returns the normalized u value and the lower-bound time index.
      std::pair<double,int> ui = computeUAndTIndex(t);
      Eigen::VectorXd u = computeU(ui.first, ui.second, derivativeOrder);
      
      int bidx = ui.second - splineOrder_ + 1;

      // Evaluate the spline (or derivative) in matrix form.
      //
      // [c_0 c_1 c_2 c_3] * B^T * u
      // spline coefficients      

      // The spline value
      Eigen::VectorXd Bt_u = basisMatrices_[bidx].transpose() * u;
      Eigen::VectorXd v = coefficients_.block(0,bidx,coefficients_.rows(),splineOrder_) * Bt_u; 

      if(Jacobian)
	{
	  // The Jacobian
	  Jacobian->resize(coefficients_.rows(), Bt_u.size() * coefficients_.rows());
	  Eigen::MatrixXd one = Eigen::MatrixXd::Identity(coefficients_.rows(), coefficients_.rows());
	  for(int i = 0; i < Bt_u.size(); i++)
	    {
	      Jacobian->block(0, i*coefficients_.rows(), coefficients_.rows(), coefficients_.rows()) = one * Bt_u[i];
	    }
	}

      if(coefficientIndices)
	{
	  int D = coefficients_.rows();
	  *coefficientIndices = Eigen::VectorXi::LinSpaced(splineOrder_*D,bidx*D,(bidx + splineOrder_)*D - 1);
	}
      return v;

    }

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> BSpline::evalDAndJacobian(double t, int derivativeOrder) const
    {
      std::pair<Eigen::VectorXd, Eigen::MatrixXd> rv;

      rv.first = evalDAndJacobian(t, derivativeOrder, &rv.second, NULL);
      
      return rv;

    }


    Eigen::MatrixXd BSpline::localBasisMatrix(double t, int derivativeOrder) const
    {
      return Phi(t,derivativeOrder);
    }

    Eigen::MatrixXd BSpline::localCoefficientMatrix(double t) const
    {
      std::pair<double,int> ui = computeTIndex(t);
      int bidx = ui.second - splineOrder_ + 1;
      return coefficients_.block(0,bidx,coefficients_.rows(),splineOrder_);
    }

    Eigen::VectorXd BSpline::localCoefficientVector(double t) const
    {

      std::pair<double,int> ui = computeTIndex(t);
      int bidx = ui.second - splineOrder_ + 1;
      Eigen::VectorXd c(splineOrder_ * coefficients_.rows());
      for(int i = 0; i < splineOrder_; i++)
	{
	  c.segment(i*coefficients_.rows(), coefficients_.rows()) = coefficients_.col(i + bidx);
	}
      return c;
    }

Eigen::VectorXd BSpline::segmentCoefficientVector(int segmentIdx) const {
  SM_ASSERT_GE_LT(std::runtime_error, segmentIdx, 0, numValidTimeSegments(), "segment index out of bounds");
  int bidx = segmentIdx;
  Eigen::VectorXd c(splineOrder_ * coefficients_.rows());
  for(int i = 0; i < splineOrder_; i++) {
    c.segment(i*coefficients_.rows(), coefficients_.rows()) = coefficients_.col(i + bidx);
  }
  return c;
}


    Eigen::VectorXi BSpline::localCoefficientVectorIndices(double t) const
    {
      std::pair<double,int> ui = computeTIndex(t);
      int bidx = ui.second - splineOrder_ + 1;
      int D = coefficients_.rows();
      return Eigen::VectorXi::LinSpaced(splineOrder_*D,bidx*D,(bidx + splineOrder_)*D - 1);
    }

Eigen::VectorXi BSpline::segmentCoefficientVectorIndices(int segmentIdx) const {
  SM_ASSERT_GE_LT(std::runtime_error, segmentIdx, 0, numValidTimeSegments(), "segment index out of bounds");
  int bidx = segmentIdx;
  int D = coefficients_.rows();
  return Eigen::VectorXi::LinSpaced(splineOrder_*D,bidx*D,(bidx + splineOrder_)*D - 1);
}

    Eigen::VectorXi BSpline::localVvCoefficientVectorIndices(double t) const
    {
      std::pair<double,int> ui = computeTIndex(t);
      int bidx = ui.second - splineOrder_ + 1;
      return Eigen::VectorXi::LinSpaced(splineOrder_,bidx,(bidx + splineOrder_) - 1);
    }

Eigen::VectorXi BSpline::segmentVvCoefficientVectorIndices(int segmentIdx) const {
  SM_ASSERT_GE_LT(std::runtime_error, segmentIdx, 0, numValidTimeSegments(), "segment index out of bounds");
  int bidx = segmentIdx;
  return Eigen::VectorXi::LinSpaced(splineOrder_,bidx,(bidx + splineOrder_) - 1);
}

    Eigen::MatrixXd BSpline::Phi(double t, int derivativeOrder) const
    {
      
      SM_ASSERT_GE(Exception, derivativeOrder, 0, "To integrate, use the integral function");
      std::pair<double,int> ui = computeUAndTIndex(t);

  //    std::cout << "  ui:" << ui.first << " " << t << std::endl;

      Eigen::VectorXd u = computeU(ui.first, ui.second, derivativeOrder);

   //   std::cout << "u:" << std::endl;
   //   std::cout << u << std::endl << std::endl;

      int bidx = ui.second - splineOrder_ + 1;
  
      
    //   std::cout << "Spline order: " << splineOrder_ << std::endl;
    //  std::cout << "t: " << t_min() << " <= " << t << " <= " << t_max() << std::endl;
    //  std::cout << "bidx: " << bidx << std::endl;
    //   std::cout << "number of basis matrices: " << basisMatrices_.size() << std::endl;
     //  std::cout << "basis matrix:\n" << basisMatrices_[bidx] << std::endl;
    //   std::cout << "u:\n" << u << std::endl;
      u = basisMatrices_[bidx].transpose() * u;
      
//      std::cout << "u:" << std::endl;
 //     std::cout << u << std::endl;


      Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(coefficients_.rows(),splineOrder_*coefficients_.rows());
      Eigen::MatrixXd one = Eigen::MatrixXd::Identity(Phi.rows(), Phi.rows());
      for(int i = 0; i < splineOrder_; i++)
	{
	  Phi.block(0,Phi.rows()*i,Phi.rows(),Phi.rows()) = one * u(i);
	}

      return Phi;
    }
    

    void BSpline::setCoefficientVector(const Eigen::VectorXd & c)
    {
      SM_ASSERT_EQ(Exception,c.size(),coefficients_.rows() * coefficients_.cols(), "The coefficient vector is the wrong size. The vector must contain all vector-valued coefficients stacked up into one column.");
      for(int i = 0; i < coefficients_.cols(); i++)
	{
	  coefficients_.col(i) = c.segment(i * coefficients_.rows(),coefficients_.rows());
	}      
    }

    Eigen::VectorXd BSpline::coefficientVector()
    {
      Eigen::VectorXd c(coefficients_.rows() * coefficients_.cols());
      for(int i = 0; i < coefficients_.cols(); i++)
	{
	  c.segment(i * coefficients_.rows(),coefficients_.rows()) = coefficients_.col(i);
	}
      return c;
    }


    void BSpline::setCoefficientMatrix(const Eigen::MatrixXd & coefficients)
    {
      SM_ASSERT_EQ(Exception,coefficients_.rows(), coefficients.rows(), "The new coefficient matrix must match the size of the existing coefficient matrix");
      SM_ASSERT_EQ(Exception,coefficients_.cols(), coefficients.cols(), "The new coefficient matrix must match the size of the existing coefficient matrix");
      coefficients_ = coefficients;
    }


    
    const Eigen::MatrixXd & BSpline::basisMatrix(int i) const
    {
      SM_ASSERT_GE_LT(Exception,i, 0, numValidTimeSegments(), "index out of range");
      return basisMatrices_[i];
    }

    
    std::pair<double,double> BSpline::timeInterval() const
    {
      return std::make_pair(t_min(), t_max());
    }
      
    std::pair<double,double> BSpline::timeInterval(int i) const
    {
      SM_ASSERT_GE(Exception, (int)knots_.size(), minimumKnotsRequired(), "The B-spline is not well initialized");
      SM_ASSERT_GE_LT(Exception, i, 0, numValidTimeSegments(), "index out of range");
      return std::make_pair(knots_[splineOrder_ + i - 1],knots_[splineOrder_ + i]);
    }

    void BSpline::initSpline(double t_0, double t_1, const Eigen::VectorXd & p_0, const Eigen::VectorXd & p_1)
    {
      SM_ASSERT_EQ(Exception,p_0.size(), p_1.size(), "The coefficient vectors should be the same size");
      SM_ASSERT_GT(Exception,t_1, t_0, "Time must be increasing from t_0 to t_1");
      
      // Initialize the spline so that it interpolates the two points and moves between them with a constant velocity.
      
      // How many knots are required for one time segment?
      int K = numKnotsRequired(1);
      // How many coefficients are required for one time segment?
      int C = numCoefficientsRequired(1);
      // What is the vector coefficient dimension
      int D = p_0.size();

      // Initialize a uniform knot sequence
      double dt = t_1 - t_0;
      std::vector<double> knots(K);
      for(int i = 0; i < K; i++)
	{
	  knots[i] = t_0 + (i - splineOrder_ + 1) * dt;
	}
      // Set the knots and zero the coefficients
      setKnotsAndCoefficients(knots, Eigen::MatrixXd::Zero(D,C));


      // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
      int coefficientDim = C * D;
      // We always need an even number of constraints. 
      int constraintsRequired = C + (C & 0x1);
      int constraintSize = constraintsRequired * D;
      
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(constraintSize, coefficientDim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(constraintSize);
      
      // Add the position constraints.
      int brow = 0;
      int bcol = 0;
      A.block(brow,bcol,D,coefficientDim) = Phi(t_min(),0);
      b.segment(brow,D) = p_0;
      brow += D;
      A.block(brow,bcol,D,coefficientDim) = Phi(t_max(),0);
      b.segment(brow,D) = p_1;
      brow += D;

      if(splineOrder_ > 2)
	{
	  // At the very minimum we have to add velocity constraints.
	  Eigen::VectorXd v = (p_1 - p_0)/dt;
	  A.block(brow,bcol,D,coefficientDim) = Phi(t_min(),1);
	  b.segment(brow,D) = v;
	  brow += D;
	  A.block(brow,bcol,D,coefficientDim) = Phi(t_max(),1);
	  b.segment(brow,D) = v;
	  brow += D;
	  
	  if(splineOrder_ > 4)
	    {
	      // Now we add the constraint that all higher-order derivatives are zero.
	      int derivativeOrder = 2;
	      Eigen::VectorXd z = Eigen::VectorXd::Zero(D);
	      while(brow < A.rows())
		{
		  A.block(brow,bcol,D,coefficientDim) = Phi(t_min(),derivativeOrder);
		  b.segment(brow,D) = z;
		  brow += D;
		  A.block(brow,bcol,D,coefficientDim) = Phi(t_max(),derivativeOrder);
		  b.segment(brow,D) = z;
		  brow += D;
		  ++derivativeOrder;
		}
	    }
	}

      // Now we solve the Ax=b system
      if(A.rows() != A.cols())
	{
	  // The system is over constrained. This happens for odd ordered splines.
	  b = (A.transpose() * b).eval();
	  A = (A.transpose() * A).eval();
	}
      
      // Solve for the coefficient vector.
      Eigen::VectorXd c = A.householderQr().solve(b);
      // ldlt doesn't work for this problem. It may be because the ldlt decomposition
      // requires the matrix to be positive or negative semidefinite
      // http://eigen.tuxfamily.org/dox-devel/TutorialLinearAlgebra.html#TutorialLinAlgRankRevealing
      // which may imply that it is symmetric. Our A matrix is only symmetric in the over-constrained case.
      //Eigen::VectorXd c = A.ldlt().solve(b);
      setCoefficientVector(c);
    }

    void BSpline::addCurveSegment(double t, const Eigen::VectorXd & p_1)
    {
      SM_ASSERT_GT(Exception, t, t_max(), "The new time must be past the end of the last valid segment");
      SM_ASSERT_EQ(Exception, p_1.size(), coefficients_.rows(), "Invalid coefficient vector size");
      
      // Get the final valid time interval.
      int NT = numValidTimeSegments();
      std::pair<double, double> interval_km1 = timeInterval(NT-1);

      Eigen::VectorXd p_0;
      
      // Store the position of the spline at the  end of the interval.
      // We will use these as constraints as we don't want them to change.
      p_0 = eval(interval_km1.second);
      
      // Retool the knot vector.
      double du;
      int km1;
      boost::tie(du,km1) = computeTIndex(interval_km1.first);
      
      // leave knots km1 and k alone but retool the other knots.
      double dt = t - knots_[km1 + 1];
      double kt = t;
      
      // add another knot.
      std::vector<double> knots(knots_);
      knots.push_back(0.0);
      // space the further knots uniformly.
      for(unsigned k = km1 + 2; k < knots.size(); k++)
	{
	  knots[k] = kt;
	  kt += dt;
	}
      // Tack on an new, uninitialized coefficient column.
      Eigen::MatrixXd c(coefficients_.rows(), coefficients_.cols() + 1);
      c.topLeftCorner(coefficients_.rows(), coefficients_.cols()) = coefficients_;
      setKnotsAndCoefficients(knots,c);
      
      // Now, regardless of the order of the spline, we should only have to add a single knot and coefficient vector.
      // In this case, we should solve for the last two coefficient vectors (i.e., the new one and the one before the
      // new one).
      
      // Get the time interval of the new time segment.
      double t_0, t_1;
      boost::tie(t_0,t_1) = timeInterval(NT);

      // what is the coefficient dimension?
      int D = coefficients_.rows();
      // How many vector-valued coefficients are required? In this case, 2. We will leave the others fixed.
      int C = 2;
      // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
      int coefficientDim = C * D;
      // We always need an even number of constraints. 
      int constraintsRequired = 2;
      int constraintSize = constraintsRequired * D;
      
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(constraintSize, coefficientDim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(constraintSize);      // Build the A matrix.

      int phiBlockColumnOffset = D * std::max(0,(splineOrder_ - 2));
      Eigen::VectorXd fixedCoefficients = localCoefficientVector(t_0).segment(0,phiBlockColumnOffset);

      // Add the position constraints.
      int brow = 0;
      int bcol = 0;
      Eigen::MatrixXd P;
      P = Phi(t_0,0);
      A.block(brow,bcol,D,coefficientDim) = P.block(0,phiBlockColumnOffset, D, coefficientDim);
      b.segment(brow,D) = p_0 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;
      brow += D;

      P = Phi(t_1,0);
      A.block(brow,bcol,D,coefficientDim) = P.block(0,phiBlockColumnOffset, D, coefficientDim);
      b.segment(brow,D) = p_1 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;;
      brow += D;

      // Add regularization constraints (keep the coefficients small)
      //A.block(brow,bcol,coefficientDim,coefficientDim) = 1e-4 * Eigen::MatrixXd::Identity(coefficientDim, coefficientDim);
      //b.segment(brow,coefficientDim) = Eigen::VectorXd::Zero(coefficientDim);
      //brow += coefficientDim;


      // Now we solve the Ax=b system
      if(A.rows() != A.cols())
	{
	  // The system is over constrained. This happens for odd ordered splines.
	  b = (A.transpose() * b).eval();
	  A = (A.transpose() * A).eval();
	}

      
      Eigen::VectorXd cstar = A.householderQr().solve(b);
      coefficients_.col(coefficients_.cols() - 2) = cstar.head(D);
      coefficients_.col(coefficients_.cols() - 1) = cstar.tail(D);

    }

    
    void BSpline::removeCurveSegment()
    {
      if(knots_.size() > 0 && coefficients_.cols() > 0)
	{
	  knots_.erase(knots_.begin());
	  coefficients_ = coefficients_.block(0,1,coefficients_.rows(),coefficients_.cols() - 1).eval();
	}
    }

    void BSpline::setLocalCoefficientVector(double t, const Eigen::VectorXd & c)
    {
      SM_ASSERT_EQ(Exception, c.size(), splineOrder_ * coefficients_.rows(), "The local coefficient vector is the wrong size");
      std::pair<double,int> ui = computeTIndex(t);
      int bidx = ui.second - splineOrder_ + 1;
      for(int i = 0; i < splineOrder_; i++)
	{
	  coefficients_.col(i + bidx) = c.segment(i*coefficients_.rows(), coefficients_.rows());
	}

    }


    void BSpline::initSpline2(const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda)
    {
      SM_ASSERT_EQ(Exception,times.size(), interpolationPoints.cols(), "The number of times and the number of interpolation points must be equal");
      SM_ASSERT_GE(Exception,times.size(),2, "There must be at least two times");
      SM_ASSERT_GE(Exception,numSegments,1, "There must be at least one time segment");
      for(int i = 1; i < times.size(); i++)
	{
	  SM_ASSERT_LE(Exception, times[i-1], times[i],
			 "The time sequence must be nondecreasing. time " << i
			 << " was not greater than or equal to time " << (i-1));
	}
      
      
      // Initialize the spline so that it interpolates the N points

      // How many knots are required for one time segment?
      int K = numKnotsRequired(numSegments);
      // How many coefficients are required for one time segment?
      int C = numCoefficientsRequired(numSegments);
      // What is the vector coefficient dimension
      int D = interpolationPoints.rows();

      // Initialize a uniform knot sequence
      double dt = (times[times.size() - 1] - times[0]) / numSegments;
      std::vector<double> knots(K);
      for(int i = 0; i < K; i++)
	{
	  knots[i] = times[0] + (i - splineOrder_ + 1) * dt;
	}
      // Set the knots and zero the coefficients
      setKnotsAndCoefficients(knots, Eigen::MatrixXd::Zero(D,C));


      // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
      int coefficientDim = C * D;
      
      int numConstraints = (knots.size() - 2 * splineOrder_ + 2) + interpolationPoints.cols();
      int constraintSize = numConstraints * D;
      
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(constraintSize, coefficientDim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(constraintSize);

      int brow = 0;
      //int bcol = 0;
      // Now add the regularization constraint.
      //A.block(brow,bcol,coefficientDim,coefficientDim) = 1e-1* Eigen::MatrixXd::Identity(coefficientDim, coefficientDim);
      //b.segment(brow,coefficientDim) = Eigen::VectorXd::Zero(coefficientDim);
      //brow += coefficientDim;
      for(int i = splineOrder_ - 1; i < (int)knots.size() - splineOrder_ + 1; i++)
	{
	  Eigen::VectorXi coeffIndices = localCoefficientVectorIndices(knots[i]);
	  
	  A.block(brow,coeffIndices[0],D,coeffIndices.size()) = lambda * Phi(knots[i],2);
	  b.segment(brow,D) = Eigen::VectorXd::Zero(D);
	  brow += D;
	}

      // Add the position constraints.
      for(int i = 0; i < interpolationPoints.cols(); i++)
	{
	  Eigen::VectorXi coeffIndices = localCoefficientVectorIndices(times[i]);
	  A.block(brow,coeffIndices[0],D,coeffIndices.size()) = Phi(times[i],0);
	  
	  b.segment(brow,D) = interpolationPoints.col(i);
	  brow += D;
	}

      // Now we solve the Ax=b system
      //if(A.rows() != A.cols())
      //	{
	  // The system is over constrained. This happens for odd ordered splines.
	  b = (A.transpose() * b).eval();
	  A = (A.transpose() * A).eval();
	  //	}
      
      // Solve for the coefficient vector.
      Eigen::VectorXd c = A.ldlt().solve(b);
      // ldlt doesn't work for this problem. It may be because the ldlt decomposition
      // requires the matrix to be positive or negative semidefinite
      // http://eigen.tuxfamily.org/dox-devel/TutorialLinearAlgebra.html#TutorialLinAlgRankRevealing
      // which may imply that it is symmetric. Our A matrix is only symmetric in the over-constrained case.
      // Eigen::VectorXd c = A.ldlt().solve(b);
      setCoefficientVector(c);
    }

    
    void BSpline::initSplineSparseKnots(const Eigen::VectorXd &times, const Eigen::MatrixXd &interpolationPoints, const Eigen::VectorXd knots, double lambda)
    {
        
    	SM_ASSERT_EQ(Exception,times.size(), interpolationPoints.cols(), "The number of times and the number of interpolation points must be equal");
    	SM_ASSERT_GE(Exception,times.size(),2, "There must be at least two times");
    	for(int i = 1; i < times.size(); i++)
    	{
    		SM_ASSERT_LE(Exception, times[i-1], times[i],
                         "The time sequence must be nondecreasing. time " << i
                         << " was not greater than or equal to time " << (i-1));
    	}
        
    	int K = knots.size();
    	// How many coefficients are required for one time segment?
    	int C = numCoefficientsRequired(knots.size() - 2*(splineOrder_ - 1)-1);
    	// What is the vector coefficient dimension
    	int D = interpolationPoints.rows();
        
    	// Set the knots and zero the coefficients
    	std::vector<double> knotsVector(K);
    	for(int i = 0; i < K; i++)
    	{
    		knotsVector[i] = knots(i);
    	}
    	setKnotsAndCoefficients(knotsVector, Eigen::MatrixXd::Zero(D,C));
        
    	// define the structure:
    	std::vector<int> rows;
    	std::vector<int> cols;
        
    	for (int i = 1; i <= interpolationPoints.cols(); i++)
    		rows.push_back(i*D);
    	for(int i = 1; i <= C; i++)
    		cols.push_back(i*D);
        
    	std::vector<int> bcols(1);
    	bcols[0] = 1;
        
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> A(rows,cols, true);
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> b(rows,bcols, true);
        
    	int brow = 0;
    	// try to fill the matrix:
    	for(int i = 0; i < interpolationPoints.cols(); i++) {
    		Eigen::VectorXi coeffIndices = localCoefficientVectorIndices(times[i]);
            
    		const bool allocateBlock = true;
            
    		// get Phi
    		Eigen::MatrixXd P = Phi(times[i],0); // Dx(n*D)
            
    		// the n'th order spline needs n column blocks (n*D columns)
    		for(int j = 0; j < splineOrder_; j++) {
    			Eigen::MatrixXd & Ai = *A.block(brow/D,coeffIndices[0]/D+j,allocateBlock );
    			Ai= P.block(0,j*D,D,D);
    		}
            
    		Eigen::MatrixXd & bi = *b.block(brow/D,0,allocateBlock );
    		bi = interpolationPoints.col(i);
            
    		brow += D;
    	}
        
    	//Eigen::MatrixXd Ad = A.toDense();
        
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> At(cols,rows, true);
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> * Atp = &At;
    	A.transpose(Atp);
        
    	// A'b
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Ab(cols,bcols, true);
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> * Abp = &Ab;
    	Atp->multiply(Abp, &b);
        
    	// A'A
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> AtA(cols,cols, true);
    	sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> * AtAp = &AtA;
    	Atp->multiply(AtAp, &A);
        
    	// Add the motion constraint.
    	Eigen::VectorXd W = Eigen::VectorXd::Constant(D,lambda);
        
        // make this conditional on the order of the spline:
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Q(cols,cols,true);
        if (splineOrder_ == 2)
            curveQuadraticIntegralDiagSparse(W, 1).cloneInto(Q);
        else
            curveQuadraticIntegralDiagSparse(W, 2).cloneInto(Q);

        
    	// A'A + Q
    	Q.add(AtAp);
        
    	// solve:
    	sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd> solver;
    	solver.init();
        
    	Eigen::VectorXd c(AtAp->rows());
    	c.setZero();
    	Eigen::VectorXd b_dense = Abp->toDense();
        
    	bool result = solver.solve(*AtAp,&c[0],&b_dense[0]);
    	if(!result) {
    		c.setZero();
    		// fallback => use nonsparse solver:
    		std::cout << "Fallback to Dense Solver" << std::endl;
    		Eigen::MatrixXd Adense = AtAp->toDense();
    		c = Adense.ldlt().solve(b_dense);
    	}
        
    	//      std::cout << "b\nA=" << A << "\n b=" << b << "\n";
        
    	// Solve for the coefficient vector.
    	//   Eigen::VectorXd c = A.ldlt().solve(b);
    	setCoefficientVector(c);
    }
    

    void BSpline::initSplineSparse(const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda)
    {
        SM_ASSERT_EQ(Exception,times.size(), interpolationPoints.cols(), "The number of times and the number of interpolation points must be equal");
        SM_ASSERT_GE(Exception,times.size(),2, "There must be at least two times");
        SM_ASSERT_GE(Exception,numSegments,1, "There must be at least one time segment");
        for(int i = 1; i < times.size(); i++)
        {
            SM_ASSERT_LE(Exception, times[i-1], times[i],
                         "The time sequence must be nondecreasing. time " << i
                         << " was not greater than or equal to time " << (i-1));
        }

        
        // How many knots are required for one time segment?
        int K = numKnotsRequired(numSegments);
        // How many coefficients are required for one time segment?
        int C = numCoefficientsRequired(numSegments);
        // What is the vector coefficient dimension
        int D = interpolationPoints.rows();
        
        // Initialize a uniform knot sequence
        double dt = (times[times.size() - 1] - times[0]) / numSegments;
        std::vector<double> knots(K);
        for(int i = 0; i < K; i++)
        {
            knots[i] = times[0] + (i - splineOrder_ + 1) * dt;
        }
        // Set the knots and zero the coefficients
        setKnotsAndCoefficients(knots, Eigen::MatrixXd::Zero(D,C));
        
        // define the structure:
        std::vector<int> rows;
        std::vector<int> cols;
        
        for (int i = 1; i <= interpolationPoints.cols(); i++)
            rows.push_back(i*D);
        for(int i = 1; i <= C; i++)
            cols.push_back(i*D);
 
        
        std::vector<int> bcols(1);
        bcols[0] = 1;
        
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> A(rows,cols, true);
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> b(rows,bcols, true);
        
        int brow = 0;
        // try to fill the matrix:
        for(int i = 0; i < interpolationPoints.cols(); i++) {
            Eigen::VectorXi coeffIndices = localCoefficientVectorIndices(times[i]);

            const bool allocateBlock = true;
            
            // get Phi
            Eigen::MatrixXd P = Phi(times[i],0); // Dx(n*D)

            // the n'th order spline needs n column blocks (n*D columns)
            for(int j = 0; j < splineOrder_; j++) {
                Eigen::MatrixXd & Ai = *A.block(brow/D,coeffIndices[0]/D+j,allocateBlock );
                Ai= P.block(0,j*D,D,D);
            }
            
            Eigen::MatrixXd & bi = *b.block(brow/D,0,allocateBlock );
            bi = interpolationPoints.col(i);
            
            brow += D;
        }

        //Eigen::MatrixXd Ad = A.toDense();

        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> At(cols,rows, true);
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> * Atp = &At;
        A.transpose(Atp);
        
        // A'b
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Ab(cols,bcols, true);
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> * Abp = &Ab;
        Atp->multiply(Abp, &b);
        
        // A'A
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> AtA(cols,cols, true);
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> * AtAp = &AtA;
        Atp->multiply(AtAp, &A);

        // Add the motion constraint.
        Eigen::VectorXd W = Eigen::VectorXd::Constant(D,lambda);
        
        // make this conditional on the order of the spline:
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Q(cols,cols,true);
        if (splineOrder_ == 2)
            curveQuadraticIntegralDiagSparse(W, 1).cloneInto(Q);
        else
            curveQuadraticIntegralDiagSparse(W, 2).cloneInto(Q);
  
        // A'A + Q
        Q.add(AtAp);
        
        // solve:
        sparse_block_matrix::LinearSolverCholmod<Eigen::MatrixXd> solver;
        solver.init();
        
        Eigen::VectorXd c(AtAp->rows());
        c.setZero();
        Eigen::VectorXd b_dense = Abp->toDense();

        bool result = solver.solve(*AtAp,&c[0],&b_dense[0]);
        if(!result) {
            c.setZero();
            // fallback => use nonsparse solver:
            std::cout << "Fallback to Dense Solver" << std::endl;
            Eigen::MatrixXd Adense = AtAp->toDense();
            c = Adense.ldlt().solve(b_dense);
        }

        //      std::cout << "b\nA=" << A << "\n b=" << b << "\n";
        
        // Solve for the coefficient vector.
     //   Eigen::VectorXd c = A.ldlt().solve(b);
        setCoefficientVector(c);         

    }
    
    
    
    void BSpline::initSpline3(const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda)
    {
      SM_ASSERT_EQ(Exception,times.size(), interpolationPoints.cols(), "The number of times and the number of interpolation points must be equal");
      SM_ASSERT_GE(Exception,times.size(),2, "There must be at least two times");
      SM_ASSERT_GE(Exception,numSegments,1, "There must be at least one time segment");
      for(int i = 1; i < times.size(); i++)
	{
	  SM_ASSERT_LE(Exception, times[i-1], times[i],
			 "The time sequence must be nondecreasing. time " << i
			 << " was not greater than or equal to time " << (i-1));
	}

      // How many knots are required for one time segment?
      int K = numKnotsRequired(numSegments);
      // How many coefficients are required for one time segment?
      int C = numCoefficientsRequired(numSegments);
      // What is the vector coefficient dimension
      int D = interpolationPoints.rows();

      // Initialize a uniform knot sequence
      double dt = (times[times.size() - 1] - times[0]) / numSegments;
      std::vector<double> knots(K);
      for(int i = 0; i < K; i++)
	{
	  knots[i] = times[0] + (i - splineOrder_ + 1) * dt;
	}
      // Set the knots and zero the coefficients
      setKnotsAndCoefficients(knots, Eigen::MatrixXd::Zero(D,C));


      // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
      int coefficientDim = C * D;
      
      int numConstraints = interpolationPoints.cols();
      int constraintSize = numConstraints * D;
      
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(constraintSize, coefficientDim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(constraintSize);
        
   //     std::cout << A.rows() << ":" << A.cols() << std::endl;
        

      int brow = 0;
      // Add the position constraints.
      for(int i = 0; i < interpolationPoints.cols(); i++)
	{
	  Eigen::VectorXi coeffIndices = localCoefficientVectorIndices(times[i]);

    //    std::cout << brow << ":" << coeffIndices[0] << std::endl;
        
	  A.block(brow,coeffIndices[0],D,coeffIndices.size()) = Phi(times[i],0);

	  b.segment(brow,D) = interpolationPoints.col(i);
	  brow += D;
	}


   //   std::cout << b << std::endl;

      b = (A.transpose() * b).eval();
      A = (A.transpose() * A).eval();

      // Add the motion constraint.
      Eigen::VectorXd W = Eigen::VectorXd::Constant(D,lambda);
    
      // make this conditional on the order of the spline:
      if (splineOrder_ == 2)
          A += curveQuadraticIntegralDiag(W, 1);
      else
          A += curveQuadraticIntegralDiag(W, 2);
        
      Eigen::VectorXd c = A.ldlt().solve(b);
      setCoefficientVector(c);

    }


    void BSpline::addCurveSegment2(double t, const Eigen::VectorXd & p_1, double lambda)
    {
      SM_ASSERT_GT(Exception, t, t_max(), "The new time must be past the end of the last valid segment");
      SM_ASSERT_EQ(Exception, p_1.size(), coefficients_.rows(), "Invalid coefficient vector size");
      
      // Get the final valid time interval.
      int NT = numValidTimeSegments();
      std::pair<double, double> interval_km1 = timeInterval(NT-1);

      Eigen::VectorXd p_0;
      
      // Store the position of the spline at the  end of the interval.
      // We will use these as constraints as we don't want them to change.
      p_0 = eval(interval_km1.second);
      
      // Retool the knot vector.
      double du;
      int km1;
      boost::tie(du,km1) = computeTIndex(interval_km1.first);
      
      // leave knots km1 and k alone but retool the other knots.
      double dt = t - knots_[km1 + 1];
      double kt = t;
      
      // add another knot.
      std::vector<double> knots(knots_);
      knots.push_back(0.0);
      // space the further knots uniformly.
      for(unsigned k = km1 + 2; k < knots.size(); k++)
	{
	  knots[k] = kt;
	  kt += dt;
	}
      // Tack on an new, uninitialized coefficient column.
      Eigen::MatrixXd c(coefficients_.rows(), coefficients_.cols() + 1);
      c.topLeftCorner(coefficients_.rows(), coefficients_.cols()) = coefficients_;
      setKnotsAndCoefficients(knots,c);
      
      // Now, regardless of the order of the spline, we should only have to add a single knot and coefficient vector.
      // In this case, we should solve for the last two coefficient vectors (i.e., the new one and the one before the
      // new one).
      
      // Get the time interval of the new time segment.
      double t_0, t_1;
      boost::tie(t_0,t_1) = timeInterval(NT);

      // what is the coefficient dimension?
      int D = coefficients_.rows();
      // How many vector-valued coefficients are required? In this case, 2. We will leave the others fixed.
      int C = 2;
      // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
      int coefficientDim = C * D;
      // We always need an even number of constraints. 
      int constraintsRequired = 2 + 2;
      int constraintSize = constraintsRequired * D;
      
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(constraintSize, coefficientDim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(constraintSize);      // Build the A matrix.

      int phiBlockColumnOffset = D * std::max(0,(splineOrder_ - 2));
      Eigen::VectorXd fixedCoefficients = localCoefficientVector(t_0).segment(0,phiBlockColumnOffset);

      // Add the position constraints.
      int brow = 0;
      int bcol = 0;
      Eigen::MatrixXd P;
      P = Phi(t_0,0);
      A.block(brow,bcol,D,coefficientDim) = P.block(0,phiBlockColumnOffset, D, coefficientDim);
      b.segment(brow,D) = p_0 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;
      brow += D;

      P = Phi(t_1,0);
      A.block(brow,bcol,D,coefficientDim) = P.block(0,phiBlockColumnOffset, D, coefficientDim);
      b.segment(brow,D) = p_1 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;;
      brow += D;


      // Add regularization constraints (keep the acceleration small)
      P = Phi(t_0,2);
      A.block(brow,bcol,D,coefficientDim) = lambda * P.block(0,phiBlockColumnOffset, D, coefficientDim);
      b.segment(brow,D) = Eigen::VectorXd::Zero(D);
      brow += D;

      P = Phi(t_1,2);
      A.block(brow,bcol,D,coefficientDim) = lambda * P.block(0,phiBlockColumnOffset, D, coefficientDim);
      b.segment(brow,D) = Eigen::VectorXd::Zero(D);
      brow += D;

      //A.block(brow,bcol,coefficientDim,coefficientDim) = 1e-4 * Eigen::MatrixXd::Identity(coefficientDim, coefficientDim);
      //b.segment(brow,coefficientDim) = Eigen::VectorXd::Zero(coefficientDim);
      //brow += coefficientDim;


      // Now we solve the Ax=b system
      if(A.rows() != A.cols())
	{
	  // The system is over constrained. This happens for odd ordered splines.
	  b = (A.transpose() * b).eval();
	  A = (A.transpose() * A).eval();
	}

      
      Eigen::VectorXd cstar = A.householderQr().solve(b);
      coefficients_.col(coefficients_.cols() - 2) = cstar.head(D);
      coefficients_.col(coefficients_.cols() - 1) = cstar.tail(D);

    }


    Eigen::MatrixXd BSpline::Vi(int segmentIndex) const
    {
      SM_ASSERT_GE_LT(Exception, segmentIndex, 0, numValidTimeSegments(), "Segment index out of bounds"); 
      
      Eigen::VectorXd vals(splineOrder_*2);
      for (int i = 0; i < vals.size(); ++i)
	{
	  vals[i] = 1.0/(i + 1.0);
	}
      
      Eigen::MatrixXd V(splineOrder_,splineOrder_);
      for(int r = 0; r < V.rows(); r++)
	{
	  for(int c = 0; c < V.cols(); c++)
	    {
	      V(r,c) = vals[r + c];
	    }
	}

      double t_0,t_1;
      boost::tie(t_0,t_1) = timeInterval(segmentIndex);

      V *= t_1 - t_0;


      return V;
    }

    Eigen::VectorXd BSpline::evalIntegral(double t1, double t2) const
    {
      if(t1 > t2)
	{
	  return -evalIntegral(t2,t1);
	}

      std::pair<double,int> u1 = computeTIndex(t1);
      std::pair<double,int> u2 = computeTIndex(t2);
      
      Eigen::VectorXd integral = Eigen::VectorXd::Zero(coefficients_.rows());

      // LHS remainder.
      double lhs_remainder = t1 - knots_[u1.second];
      if(lhs_remainder > 1e-16 && u1.first > 1e-16)
	{
	  lhs_remainder /= u1.first;
	  Eigen::VectorXd v(splineOrder_);
	  double du = lhs_remainder;
	  for(int i = 0; i < splineOrder_; i++)
	    {
	      v(i) = du/(i + 1.0);
	      du *= lhs_remainder;
	    }

	  int bidx = basisMatrixIndexFromStartingKnotIndex(u1.second);
	  integral -= u1.first * coefficients_.block(0,bidx,coefficients_.rows(),splineOrder_) * basisMatrices_[bidx].transpose() * v;
	}

      // central time segments.
      Eigen::VectorXd v = Eigen::VectorXd::Zero(splineOrder_);
      for(int i = 0; i < splineOrder_; i++)
	{
	  v(i) = 1.0/(i + 1.0);
	}

      for(int s = u1.second; s < u2.second; s++)
	{
	  int bidx = basisMatrixIndexFromStartingKnotIndex(s);
	  integral += (knots_[s+1] - knots_[s]) * coefficients_.block(0,bidx,coefficients_.rows(),splineOrder_) * basisMatrices_[bidx].transpose() * v;
	}

      // RHS remainder.
      double rhs_remainder = t2 - knots_[u2.second];
      if(rhs_remainder > 1e-16 && u2.first > 1e-16)
	{
	  rhs_remainder /= u2.first;
	  
	  Eigen::VectorXd v(splineOrder_);
	  double du = rhs_remainder;
	  for(int i = 0; i < splineOrder_; i++)
	    {
	      v(i) = du / (i + 1.0);
	      du *= rhs_remainder;
	    }

	  int bidx = basisMatrixIndexFromStartingKnotIndex(u2.second);
	  integral += u2.first * coefficients_.block(0,bidx,coefficients_.rows(),splineOrder_) * basisMatrices_[bidx].transpose() * v;
	}
      

      return integral;
    }

    int BSpline::basisMatrixIndexFromStartingKnotIndex(int startingKnotIndex) const
    {
      return startingKnotIndex - splineOrder_ + 1;
    }
    int BSpline::startingKnotIndexFromBasisMatrixIndex(int basisMatrixIndex) const
    {
      return splineOrder_ + basisMatrixIndex - 1;
    }


    Eigen::MatrixXd BSpline::Bij(int segmentIndex, int columnIndex) const
    {
      SM_ASSERT_GE_LT(Exception, segmentIndex, 0, (int)basisMatrices_.size(), "Out of range");
      SM_ASSERT_GE_LT(Exception, columnIndex, 0, splineOrder_, "Out of range");
      int D = coefficients_.rows();
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(splineOrder_*D,D);
      for(int i = 0; i < D; i++)
	{
	  B.block(i*splineOrder_,i,splineOrder_,1) = basisMatrices_[segmentIndex].col(columnIndex);
	}
      return B;
    }

    Eigen::MatrixXd BSpline::Mi(int segmentIndex) const
    {
      SM_ASSERT_GE_LT(Exception, segmentIndex, 0, (int)basisMatrices_.size(), "Out of range");
      int D = coefficients_.rows();      
      Eigen::MatrixXd M = Eigen::MatrixXd::Zero(splineOrder_*D,splineOrder_*D);
      
      for(int j = 0; j < splineOrder_; j++)
	{
	  M.block(0,j*D,D*splineOrder_, D) = Bij(segmentIndex,j);
	}
      
      return M;
    }

	Eigen::VectorXd BSpline::getLocalBiVector(double t) const
	{
		Eigen::VectorXd ret = Eigen::VectorXd::Zero(splineOrder_);
		getLocalBiInto(t, ret);
		return ret;
	}

	void BSpline::getLocalBiInto(double t, Eigen::VectorXd & ret) const
	{
		int si = segmentIndex(t);
		Eigen::VectorXd lu = u(t,0);
		for(int j = 0; j < splineOrder_; j++)
		{
			ret[j] = lu.dot(basisMatrices_[si].col(j));
		}
	}


    Eigen::VectorXd BSpline::getLocalCumulativeBiVector(double t) const
    {
	    Eigen::VectorXd bi = getLocalBiVector(t);
	    int maxIndex = bi.rows() - 1;
	    // tildeB(i) = np.sum(bi[i+1:]) :
	    for(int i = 1; i <= maxIndex; i ++){
		    double sum = 0;
		    for(int j = maxIndex; j > i; j--)
			    sum += bi[j];
		    bi[i] += sum;
	    }
	    bi[0] = 1; // the sum of k successive spline basis functions is always 1
	    return bi;
    }



    int BSpline::segmentIndex(double t) const
    {
      std::pair<double,int> ui = computeTIndex(t);
      return basisMatrixIndexFromStartingKnotIndex(ui.second);
    }

    Eigen::MatrixXd BSpline::U(double t, int derivativeOrder) const
    {
      Eigen::VectorXd uvec = u(t,derivativeOrder);
      int D = coefficients_.rows();
      Eigen::MatrixXd Umat = Eigen::MatrixXd::Zero(splineOrder_ * D, D);

      for(int i = 0; i < D; i++)
	{
	  Umat.block(i*splineOrder_,i,splineOrder_,1) = uvec;
	}    

      return Umat;
    }

    Eigen::VectorXd BSpline::u(double t, int derivativeOrder) const
    {

      std::pair<double,int> ui = computeUAndTIndex(t);
      return computeU(ui.first, ui.second, derivativeOrder);
      
    }

    Eigen::MatrixXd BSpline::Di(int segmentIndex) const
    {
      int D = coefficients_.rows();
      Eigen::MatrixXd fullD = Eigen::MatrixXd::Zero(splineOrder_*D, splineOrder_*D);
    
      Eigen::MatrixXd subD = Dii(segmentIndex);

      for(int d = 0; d < D; d++)
	{
	  fullD.block(d*splineOrder_,d*splineOrder_,splineOrder_,splineOrder_) = subD;
	}

      return fullD;
    }

    Eigen::MatrixXd BSpline::Dii(int segmentIndex) const
    {
      SM_ASSERT_GE_LT(Exception, segmentIndex, 0, (int)basisMatrices_.size(), "Out of range");
      double t_0,t_1;
      boost::tie(t_0,t_1) = timeInterval(segmentIndex);
      double dt = t_1 - t_0;
      
      double recip_dt = 0.0;
      if(dt > 0)
	recip_dt = 1.0/dt;
      Eigen::MatrixXd D = Eigen::MatrixXd::Zero(splineOrder_,splineOrder_);
      for(int i = 0; i < splineOrder_ - 1; i++)
	{
	  D(i,i+1) = (i+1.0) * recip_dt;
	}

      return D;
    }

Eigen::MatrixXd BSpline::segmentIntegral(int segmentIdx, const Eigen::MatrixXd & W, int derivativeOrder) const {
  // Let's do this quick and dirty.

  auto svd = segmentQuadraticIntegral(W, segmentIdx, derivativeOrder).jacobiSvd(Eigen::ComputeFullU);
  return (svd.matrixU() * svd.singularValues().array().sqrt().matrix().asDiagonal()).transpose();
}




    Eigen::MatrixXd BSpline::segmentQuadraticIntegral(const Eigen::MatrixXd & W, int segmentIdx, int derivativeOrder) const
    {
      int D = coefficients_.rows();
      SM_ASSERT_GE_LT(Exception, segmentIdx, 0, (int)basisMatrices_.size(), "Out of range");
      SM_ASSERT_EQ(Exception,W.rows(), D, "W must be a square matrix the size of a single vector-valued coefficient");
      SM_ASSERT_EQ(Exception,W.cols(), D, "W must be a square matrix the size of a single vector-valued coefficient");

      int N = D * splineOrder_;
      Eigen::MatrixXd Q;// = Eigen::MatrixXd::Zero(N,N);
      Eigen::MatrixXd Dm = Dii(segmentIdx);
      Eigen::MatrixXd V = Vi(segmentIdx);
      Eigen::MatrixXd M = Mi(segmentIdx);
      
      // Calculate the appropriate derivative version of V
      // using the matrix multiplication version of the derivative.
      for(int i = 0; i < derivativeOrder; i++)
	{
	  V = (Dm.transpose() * V * Dm).eval();
	}

      Eigen::MatrixXd WV = Eigen::MatrixXd::Zero(N,N);
      
      for(int r = 0; r < D; r++)
	{
	for(int c = 0; c < D; c++)
	  {
	    SM_ASSERT_NEAR(Exception, W(r,c),W(c,r),1e-14,"W must be symmetric");
	    //std::cout << "Size WV: " << WV.rows() << ", " << WV.cols() << std::endl;
	    //std::cout << "Size V: " << V.rows() << ", " << V.cols() << std::endl;
	    WV.block(splineOrder_*r, splineOrder_*c,splineOrder_,splineOrder_) = W(r,c) * V;
	  }
	}
      
      Q = M.transpose() * WV * M;

      return Q;
    }

    Eigen::MatrixXd BSpline::segmentQuadraticIntegralDiag(const Eigen::VectorXd & Wdiag, int segmentIdx, int derivativeOrder) const
    {
      int D = coefficients_.rows();
      SM_ASSERT_GE_LT(Exception, segmentIdx, 0, (int)basisMatrices_.size(), "Out of range");
      SM_ASSERT_EQ(Exception,Wdiag.size(), D, "Wdiag must be the length of a single vector-valued coefficient");

      int N = D * splineOrder_;
      Eigen::MatrixXd Q;// = Eigen::MatrixXd::Zero(N,N);
      Eigen::MatrixXd Dm = Dii(segmentIdx);
      Eigen::MatrixXd V = Vi(segmentIdx);
      Eigen::MatrixXd M = Mi(segmentIdx);
      
      // Calculate the appropriate derivative version of V
      // using the matrix multiplication version of the derivative.
      for(int i = 0; i < derivativeOrder; i++)
	{
	  V = (Dm.transpose() * V * Dm).eval();
	}

      Eigen::MatrixXd WV = Eigen::MatrixXd::Zero(N,N);
      
      for(int d = 0; d < D; d++)
	{
	  //std::cout << "Size WV: " << WV.rows() << ", " << WV.cols() << std::endl;
	  //std::cout << "Size V: " << V.rows() << ", " << V.cols() << std::endl;
	  WV.block(splineOrder_*d, splineOrder_*d,splineOrder_,splineOrder_) = Wdiag(d) * V;
	}
      
      Q = M.transpose() * WV * M;

      return Q;
    }
   
    
    // sparse curveQuaddraticIntegral:
    sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> BSpline::curveQuadraticIntegralSparse(const  Eigen::MatrixXd & W, int derivativeOrder) const 
    {

        // define rows / cols:
        // blocksize:
        int D = coefficients_.rows();
        int blocksInBlock = splineOrder_;
        int blocks = numVvCoefficients();
        int matrixSize = blocks * D;
        
        std::vector<int> rows;
        std::vector<int> cols;

        int i;       
        
        for(i = D; i <= matrixSize; i+=D) {
            rows.push_back(i);
            cols.push_back(i);            
        }              
        
        // create matrix:
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Q_sparse(rows,cols,true);
        // place
        for(int s = 0; s < numValidTimeSegments(); ++s)
        {
            Eigen::MatrixXd Q = segmentQuadraticIntegral(W, s, derivativeOrder);
            // place the DxD blocks in the blocksInBlock x blocksInBlock blocks:
            for(int i = 0; i < blocksInBlock; i++) {
                for(int j = 0; j < blocksInBlock; j++) {
                    const bool allocateBlock = true;
                    Eigen::MatrixXd & Qi = *Q_sparse.block(s+i, s+j, allocateBlock);          
                    Qi += Q.block(i*D,j*D,D,D);
                }
            }

        }
        return Q_sparse;
    }
        
    
    // sparse curveQuaddraticIntegral:
    sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> BSpline::curveQuadraticIntegralDiagSparse(const Eigen::VectorXd & Wdiag, int derivativeOrder) const 
    {
        
        // define rows / cols:
        // blocksize:
        int D = coefficients_.rows();
        int blocksInBlock = splineOrder_;
        int blocks = numVvCoefficients();
        int matrixSize = blocks * D;
        
        std::vector<int> rows;
        std::vector<int> cols;
        
        int i;       
        
        for(i = D; i <= matrixSize; i+=D) {
            rows.push_back(i);
            cols.push_back(i);            
        }              
        
        // create matrix:
        sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> Q_sparse(rows,cols,true);
        // place
        for(int s = 0; s < numValidTimeSegments(); ++s)
        {
            Eigen::MatrixXd Q = segmentQuadraticIntegralDiag(Wdiag, s, derivativeOrder);
            // place the DxD blocks in the blocksInBlock x blocksInBlock blocks:
            for(int i = 0; i < blocksInBlock; i++) {
                for(int j = 0; j < blocksInBlock; j++) {
                    const bool allocateBlock = true;
                    Eigen::MatrixXd & Qi = *Q_sparse.block(s+i, s+j, allocateBlock);          
                    Qi += Q.block(i*D,j*D,D,D);
                }
            }
            
        }
        return Q_sparse;
  
    }

    
    

    Eigen::MatrixXd BSpline::curveQuadraticIntegral(const Eigen::MatrixXd & W, int derivativeOrder) const
    {
      int D = coefficients_.rows();
      SM_ASSERT_EQ(Exception,W.rows(), D, "W must be a square matrix the size of a single vector-valued coefficient");
      SM_ASSERT_EQ(Exception,W.cols(), D, "W must be a square matrix the size of a single vector-valued coefficient");
      int N = coefficients_.cols();

      Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(D*N, D*N);

      int QiSize = splineOrder_ * D; 
      for(int s = 0; s < numValidTimeSegments(); s++)
	{
	  Q.block(s*D,s*D,QiSize,QiSize) += segmentQuadraticIntegral(W, s, derivativeOrder);
	}
      

      return Q;
    }
    
    

    Eigen::MatrixXd BSpline::curveQuadraticIntegralDiag(const Eigen::VectorXd & Wdiag, int derivativeOrder) const
    {
      int D = coefficients_.rows();
      SM_ASSERT_EQ(Exception,Wdiag.size(), D, "Wdiag must be the length of a single vector-valued coefficient");
      int N = coefficients_.cols();

      Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(D*N, D*N);

      int QiSize = splineOrder_ * D; 
      for(int s = 0; s < numValidTimeSegments(); s++)
	{
	  Q.block(s*D,s*D,QiSize,QiSize) += segmentQuadraticIntegralDiag(Wdiag, s, derivativeOrder);
	}
      

      return Q;
    }

    
    
    
    

    int BSpline::coefficientVectorLength() const
    {
      return coefficients_.rows() * coefficients_.cols();
    }
    
    void BSpline::initConstantSpline(double t_min, double t_max, int numSegments, const Eigen::VectorXd & constant)
    {
      SM_ASSERT_GT(Exception,t_max,t_min, "The max time is less than the min time");
      SM_ASSERT_GE(Exception,numSegments,1, "There must be at least one segment");
      SM_ASSERT_GE(Exception, constant.size(), 1, "The constant vector must be of at least length 1");

      int K = numKnotsRequired(numSegments);
      int C = numCoefficientsRequired(numSegments);
      double dt = (t_max - t_min) / (double)numSegments;
      
      double minTime = t_min - (splineOrder_ - 1)*dt;
      double maxTime = t_max + (splineOrder_ - 1)*dt;
      Eigen::VectorXd knotVector = Eigen::VectorXd::LinSpaced(K,minTime,maxTime);
      // std::cout << "K: " << K << std::endl;
      // std::cout << "S: " << numSegments << std::endl;
      // std::cout << "segTime: " << t_min << ", " << t_max << std::endl;
      // std::cout << "dt: " << dt << std::endl;
      // std::cout << "time: " << minTime << ", " << maxTime << std::endl;
      // std::cout << "order: " << splineOrder_ << std::endl;
      // std::cout << knotVector.transpose() << std::endl;
      Eigen::MatrixXd coeff(constant.size(),C);
      for(int i = 0; i < C; i++)
	coeff.col(i) = constant;

      setKnotVectorAndCoefficients(knotVector,coeff);
    }
    
    
    int BSpline::numCoefficients() const
    {
      return coefficients_.rows() * coefficients_.cols();
    }

    Eigen::Map<Eigen::VectorXd> BSpline::vvCoefficientVector(int i)
    {
      SM_ASSERT_GE_LT(Exception, i, 0,  coefficients_.cols(), "Index out of range");
      return Eigen::Map<Eigen::VectorXd>(&coefficients_(0,i),coefficients_.rows());
    }
    
    Eigen::Map<const Eigen::VectorXd> BSpline::vvCoefficientVector(int i) const
    {
      SM_ASSERT_GE_LT(Exception, i, 0, coefficients_.cols(), "Index out of range");
      return Eigen::Map<const Eigen::VectorXd>(&coefficients_(0,i),coefficients_.rows());
    }

    int BSpline::numVvCoefficients() const
    {
      return coefficients_.cols();
    }

  } // namespace bsplines
