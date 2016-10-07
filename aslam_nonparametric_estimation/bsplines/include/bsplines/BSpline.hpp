/**
 * @file   BSpline.hpp
 * @author Paul Furgale <paul.furgale@utoronto.ca>
 * @date   Fri Feb 11 13:51:57 2011
 * 
 * @brief  A class to facilitate state estimation for vehicles in 3D
 *         space using B-splines.
 * 
 * 
 */


#ifndef _BSPLINE_HPP
#define _BSPLINE_HPP
#include <sparse_block_matrix/sparse_block_matrix.h>
#include <vector>
#include <Eigen/Core>
#include <sm/assert_macros.hpp>

namespace bsplines {
	class BiVector;
}

namespace Eigen {
	namespace internal {
		template<>
		struct functor_traits<bsplines::BiVector>
		{
			enum { Cost = 1, PacketAccess = false, IsRepeatable = true };
		};
	}
}

namespace bsplines {

	class BiVector {
		enum { Cost = 1, PacketAccess = false, IsRepeatable = true };
	private:
		const int startIndex_;
	      const double endValue_;
	      const Eigen::VectorXd localBi_;

	public:
	      BiVector(int startIndex, const Eigen::VectorXd & localBi, double endValue) : startIndex_(startIndex), endValue_(endValue), localBi_(localBi){};

	      double operator() (int i, int j = 0) const {
            // kill unused parameter warning
            static_cast<void>(j);
		      i -= startIndex_;
		      if(i < 0){
			      return endValue_;
		      }
		      if(i >= (localBi_.rows())){
			      return 0;
		      }
		      return  i >= 0 ? localBi_(i): 0;
	      }
	};

    /**
     * @class BSpline
     *
     * A class to facilitate state estimation for vechicles in 3D space using B-Splines
     * 
     */
    class BSpline
    {
    public:
      /**
       * @class Exception
       * 
       * A base class for BSpline exceptions.
       * 
       */
      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      /** 
       * Create a spline of the specified order. The resulting B-spline will
       * be a series of piecewise polynomials of degree splineOrder - 1.
       * 
       * @param splineOrder The order of the spline.
       */
      BSpline(int splineOrder);

      /** 
       * A destructor.
       * 
       */
      ~BSpline();
      
      /** 
       * 
       * @return The order of the spline
       */
      int splineOrder() const;
      
      /** 
       * 
       * @return The degree of polynomial used by the spline.
       */
      int polynomialDegree() const;

      /** 
       * 
       * @return the minimum number of knots required to have at least one valid time segment.
       */
      int minimumKnotsRequired() const;

      /**
       * @param numTimeSegments the number of time segments required
       * @return the number of coefficients required for a specified number of valid time segments.
       */
      int numCoefficientsRequired(int numTimeSegments) const;


      /**
       * @param numTimeSegments the number of time segments required
       * @return the number of knots required for a specified number of valid time segments.
       */
      int numKnotsRequired(int numTimeSegments) const;

      /** 
       * 
       * @param numKnots the number of knots.
       * @return the number of valid time segments for a given number of knots.
       */
      int numValidTimeSegments(int numKnots) const;


      /** 
       * 
       * @return the number of valid time segments for a given for the current knot sequence.
       */
      int numValidTimeSegments() const;
      

      /** 
       * Return the basis matrix active on the \f$i^{\textup{th}}\f$ time segment.
       * 
       * @param i The index of the time segment.
       * 
       * @return The basis matrix active on the time segment.
       */
      const Eigen::MatrixXd & basisMatrix(int i) const;

      /** 
       * 
       * @return the time interval that the spline is well-defined on [t_min(), t_max()]
       */
      std::pair<double,double> timeInterval() const;
      
      /** 
       * Return the time interval of a single spline segment.
       *
       * @param i The index of the time segment
       * 
       * @return the time interval of the ith spline segment.
       */
      std::pair<double,double> timeInterval(int i) const;

      /** 
       * Set the knots and coefficients of the spline. Each column of the coefficient matrix
       * is interpreted as a single, vector-valued spline coefficient. 
       * 
       * @param knots        A non-decreasing knot sequence.
       * @param coefficients A set of spline coefficients.
       */
      void setKnotsAndCoefficients(const std::vector<double> & knots, const Eigen::MatrixXd & coefficients);

      /** 
       * Set the knots and coefficients of the spline. Each column of the coefficient matrix
       * is interpreted as a single, vector-valued spline coefficient. 
       * 
       * @param knots        A non-decreasing knot sequence.
       * @param coefficients A set of spline coefficients.
       */
      void setKnotVectorAndCoefficients(const Eigen::VectorXd & knots, const Eigen::MatrixXd & coefficients);

      /** 
       * Sets the coefficient matrix from the stacked vector of coefficients.
       * 
       * @param coefficients The stacked vector of coefficients.
       */
      void setCoefficientVector(const Eigen::VectorXd & coefficients);
      
      /** 
       * 
       * @return The stacked vector of coefficients.
       */
      Eigen::VectorXd coefficientVector();

      /** 
       * Sets the matrix of coefficients.
       * 
       * @param coefficients 
       */
      void setCoefficientMatrix(const Eigen::MatrixXd & coefficients);
   

      /** 
       * @return the current knot vector.
       */
      const std::vector<double> knots() const;
      
      /** 
       * @return the current knot vector.
       */
      Eigen::VectorXd knotVector() const;

      /** 
       * @return the current spline coefficient matrix. Each column of the coefficient matrix
       * is interpreted as a single, vector-valued spline coefficient.
       */
      const Eigen::MatrixXd & coefficients() const;

      /** 
       * 
       * @return The number of total coefficients the spline currently uses
       */
      int numCoefficients() const;
      
      /** 
       * This is equivalent to spline.coefficients().cols() 
       *
       * @return The number of vector-valued coefficient columns the spline currently uses
       */
      int numVvCoefficients() const;
      

      /** 
       * 
       * @return The minimum time that the spline is well-defined on.
       */
      double t_min() const;

      /** 
       * 
       * @return The maximum time that the spline is well-defined on. Because B-splines
       *         are defined on half-open intervals, the spline curve is well defined up
       *         to but not including this time.
       */
      double t_max() const;

      /** 
       * Evaluate the spline curve at the time t.
       * 
       * @param t The time to evaluate the spline curve
       * 
       * @return The value of the spline curve at the time t.
       */
      Eigen::VectorXd eval(double t) const;

      /** 
       * Evaluate the derivative of the spline curve at time t.
       * 
       * @param t The time to evaluate the spline derivative.
       * @param derivativeOrder The order of the derivative. This must be >= 0
       * 
       * @return The value of the derivative of the spline curve evaluated at t.
       */
      Eigen::VectorXd evalD(double t, int derivativeOrder) const;

      /** 
       * Evaluate the derivative of the spline curve at time t and retrieve the Jacobian
       * of the value with respect to small changes in the paramter vector. The Jacobian
       * only refers to the local parameter vector. The indices of the local parameters with
       * respect to the full paramter vector can be retrieved using localCoefficientVectorIndices().
       * 
       * @param t The time to evaluate the spline derivative.
       * @param derivativeOrder The order of the derivative. This must be >= 0
       * 
       * @return The value of the derivative of the spline curve evaluated at t and the Jacobian.
       */
      std::pair<Eigen::VectorXd, Eigen::MatrixXd> evalDAndJacobian(double t, int derivativeOrder) const;

      /** 
       * Evaluate the derivative of the spline curve at time t and retrieve the Jacobian
       * of the value with respect to small changes in the paramter vector. The Jacobian
       * only refers to the local parameter vector. 
       * 
       * @param t The time to evaluate the spline derivative.
       * @param derivativeOrder The order of the derivative. This must be >= 0
       * @param a pointer to the Jacobian matrix to fill in
       * @param a pointer to an int vector that will be filled with the local coefficient indices
       * 
       * @return The value of the derivative of the spline curve evaluated at t.
       */
      Eigen::VectorXd evalDAndJacobian(double t, int derivativeOrder, Eigen::MatrixXd * Jacobian, Eigen::VectorXi * coefficientIndices) const;



      /** 
       * Get the local basis matrix evaluated at the time \f$ t \f$.
       * For vector-valued spline coefficients of dimension \f$ D \f$
       * and a B-spline of order $S$, this matrix will be \f$ D \times SD \f$
       * 
       * @param t The time to evaluate the local basis matrix.
       * @param derivativeOrder The derivative order to return (0 is no derivative)
       * 
       * @return The local basis matrix evaluated at time \f$ t \f$
       */
      Eigen::MatrixXd Phi(double t, int derivativeOrder = 0) const;

      /** 
       * Get the local basis matrix evaluated at the time \f$ t \f$.
       * For vector-valued spline coefficients of dimension \f$ D \f$
       * and a B-spline of order $S$, this matrix will be \f$ D \times SD \f$
       * 
       * @param t The time to evaluate the local basis matrix.
       * @param derivativeOrder The derivative order to return (0 is no derivative)
       * 
       * @return The local basis matrix evaluated at time \f$ t \f$
       */
      Eigen::MatrixXd localBasisMatrix(double t, int derivativeOrder = 0) const;

      /** 
       * Get the local coefficient matrix evaluated at the time \f$ t \f$.
       * For vector-valued spline coefficients of dimension \f$ D \f$
       * and a B-spline of order $S$, this matrix will be \f$ D \times S \f$.
       * Each column of the resulting matrix corresponds to a single vector-valued
       * coefficient.
       * 
       * @param t The time being queried
       * 
       * @return The local coefficient matrix active at time \f$ t \f$
       */
      Eigen::MatrixXd localCoefficientMatrix(double t) const;

      
      /** 
       * Return a map to a single coefficient column.
       * This allows the user to pass around what is essentially a pointer 
       * to a single column in the coefficient matrix.
       * 
       * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
       * 
       * @return A map to column i of the coefficient matrix.
       */
      Eigen::Map<Eigen::VectorXd> vvCoefficientVector(int i);

      /** 
       * Return a map to a single coefficient column.
       * This allows the user to pass around what is essentially a pointer 
       * to a single column in the coefficient matrix.
       * 
       * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
       * 
       * @return A map to column i of the coefficient matrix.
       */
      Eigen::Map<const Eigen::VectorXd> vvCoefficientVector(int i) const;

      /** 
       * Return a map to a single coefficient column.
       * This allows the user to pass around what is essentially a pointer 
       * to a single column in the coefficient matrix.
       * 
       * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
       * 
       * @return A map to column i of the coefficient matrix.
       */
      template<int D>
      Eigen::Map<Eigen::Matrix<double, D, 1> > fixedSizeVvCoefficientVector(int i)
      {
	SM_ASSERT_EQ_DBG(Exception,D,coefficients_.rows(), "Size mismatch between requested vector size and actual vector size");
	SM_ASSERT_GE_LT(Exception, i, 0,  coefficients_.cols(), "Index out of range");
	return Eigen::Map<Eigen::Matrix<double, D, 1> >(&coefficients_(0,i),coefficients_.rows());

      }

      /** 
       * Return a map to a single coefficient column.
       * This allows the user to pass around what is essentially a pointer 
       * to a single column in the coefficient matrix.
       * 
       * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
       * 
       * @return A map to column i of the coefficient matrix.
       */
      template<int D>
      Eigen::Map<const Eigen::Matrix<double, D, 1> > fixedSizeVvCoefficientVector(int i) const
      {
	SM_ASSERT_EQ_DBG(Exception,D,coefficients_.rows(), "Size mismatch between requested vector size and actual vector size");
	SM_ASSERT_GE_LT(Exception, i, 0,  coefficients_.cols(), "Index out of range");
	return Eigen::Map< const Eigen::Matrix<double, D, 1> >(&coefficients_(0,i),coefficients_.rows());

      }


      /** 
       * Get the local coefficient vector evaluated at the time \f$ t \f$.
       * For vector-valued spline coefficients of dimension \f$ D \f$
       * and a B-spline of order $S$, this vector will be \f$ SD \times 1 \f$
       * Evaluating the B-spline at time t, eval(t,O) is equivalent to evaluating
       * Phi(t,O) * localCoefficientVector(t)
       * 
       * @param t The time being queried
       * 
       * @return The local coefficient vector active at time \f$ t \f$
       */
      Eigen::VectorXd localCoefficientVector(double t) const;

      /** 
       * Get the local coefficient vector for segment i
       * 
       * @param segmentIdx the segment index
       * 
       * @return The local coefficient vector active on time segment i
       */
      Eigen::VectorXd segmentCoefficientVector(int segmentIdx) const;


      /** 
       * Update the local coefficient vector
       * 
       * @param t The time used to select the local coefficients.
       * @param c The local coefficient vector.
       */
      void setLocalCoefficientVector(double t, const Eigen::VectorXd & c);

      /** 
       * Get the indices of the local coefficients active at time t.
       * 
       * @param t The time being queried.
       * 
       * @return The indices of the local coefficients active at time t.
       */
      Eigen::VectorXi localCoefficientVectorIndices(double t) const;


      /**
       * Get the indices of the local coefficients active on segment i
       *
       * @param segmentIdx The segment being queried.
       * 
       * @return The indices of the local coefficients active on this segment.
       */
      Eigen::VectorXi segmentCoefficientVectorIndices(int segmentIdx) const;

      /** 
       * Get the indices of the local vector-valued coefficients active at time t.
       * 
       * @param t The time being queried.
       * 
       * @return The indices of the local vector-valued coefficients active at time t.
       */
      Eigen::VectorXi localVvCoefficientVectorIndices(double t) const;

      /**
       * Get the indices of the local vector-valued coefficients active on segment i
       *
       * @param segmentIdx The segment being queried.
       * 
       * @return The indices of the local vector-valued coefficients active on this segment.
       */
      Eigen::VectorXi segmentVvCoefficientVectorIndices(int segmentIdx) const;


      int coefficientVectorLength() const;

      /** 
       * Initialize a spline from two times and two positions. The spline will be initialized to
       * have one valid time segment \f$[t_0, t_1)\f$ such that \f$\mathbf b(t_0) = \mathbf p_0\f$, 
       * \f$\mathbf b(t_1) = \mathbf p_1\f$, 
       * \f$\dot{\mathbf b}(t_0) = \frac{\mathbf{p_1} - \mathbf p_0}{t_1 - t_0}\f$, and 
       * \f$\dot{\mathbf b}(t_1) = \frac{\mathbf{p_1} - \mathbf p_0}{t_1 - t_0}\f$.
       * 
       * @param t_0 The start of the time interval.
       * @param t_1 The end of the time interval
       * @param p_0 The position at the start of the time interval.
       * @param p_1 The position at the end of the time interval.
       */
      void initSpline(double t_0, double t_1, const Eigen::VectorXd & p_0, const Eigen::VectorXd & p_1);
      
      void initSpline2(const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda);

      void initSpline3(const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda);
        
      void initSplineSparse(const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda);
    
      void initSplineSparseKnots(const Eigen::VectorXd &times, const Eigen::MatrixXd &interpolationPoints, const Eigen::VectorXd knots, double lambda);

        
        
      /** 
       * Add a curve segment that interpolates the point p, ending at time t.
       *
       * If the new time corresponds with the first knot past the end of the curve,
       * the existing curve is perfectly preserved. Otherwise, the existing curve
       * will interpolate its current position at the current endpoint and the new
       * position at the new endpoint but will not necessarily match the last segment
       * exactly.
       * 
       * @param t The time of the point to interpolate. This must be greater than t_max()
       * @param p The point to interpolate at time t.
       */
      void addCurveSegment(double t, const Eigen::VectorXd & p);
 

      /** 
       * Add a curve segment that interpolates the point p, ending at time t.
       *
       * If the new time corresponds with the first knot past the end of the curve,
       * the existing curve is perfectly preserved. Otherwise, the existing curve
       * will interpolate its current position at the current endpoint and the new
       * position at the new endpoint but will not necessarily match the last segment
       * exactly.
       * 
       * @param t The time of the point to interpolate. This must be greater than t_max()
       * @param p The point to interpolate at time t.
       * @param lambda a smoothness parameter. Higher for more smooth.
       */
      void addCurveSegment2(double t, const Eigen::VectorXd & p, double lambda);
      
 

      /** 
       * Removes a curve segment from the left by removing one knot and one coefficient vector.
       * After calling this function, the curve will have one fewer segment. The new minimum
       * time will be timeInterval(0).first
       * 
       */
      void removeCurveSegment();


      /** 
       * Get the \f$ \mathbf V_i \f$ matrix associated with the integral over the segment.
       * 
       * @param segmentIndex 
       * 
       * @return the \f$ \mathbf V_i \f$ matrix
       */
      Eigen::MatrixXd Vi(int segmentIndex) const;

      Eigen::VectorXd evalIntegral(double t1, double t2) const;
      inline Eigen::VectorXd evalI(double t1, double t2) const {return evalIntegral(t1,t2);}

      Eigen::MatrixXd Mi(int segmentIndex) const;
      Eigen::MatrixXd Bij(int segmentIndex, int columnIndex) const;
      Eigen::MatrixXd U(double t, int derivativeOrder) const;
      Eigen::VectorXd u(double t, int derivativeOrder) const;
      int segmentIndex(double t) const;
      Eigen::MatrixXd Dii(int segmentIndex) const;
      Eigen::MatrixXd Di(int segmentIndex) const;
      
      /**
       * Get the b_i(t) for i in localVvCoefficientVectorIndices (@see #localVvCoefficientVectorIndices).
       *
       * @param t The time being queried.
       *
       * @return [b_i(t) for i in localVvCoefficientVectorIndices].
       *
       */
      Eigen::VectorXd getLocalBiVector(double t) const;
      void getLocalBiInto(double t, Eigen::VectorXd & ret) const;

      /**
       * Get the cumulative (tilde) b_i(t) for i in localVvCoefficientVectorIndices (@see #localVvCoefficientVectorIndices).
       *
       * @param t The time being queried.
       *
       * @return [tilde b_i(t) for i in localVvCoefficientVectorIndices].
       *
       */
      Eigen::VectorXd getLocalCumulativeBiVector(double t) const;


      Eigen::CwiseNullaryOp <BiVector, Eigen::VectorXd> getBiVector(double t) const {
	      return Eigen::CwiseNullaryOp <BiVector, Eigen::VectorXd>(numValidTimeSegments(), 1, BiVector(segmentIndex(t), getLocalBiVector(t), 0));
      }
      Eigen::CwiseNullaryOp <BiVector, Eigen::VectorXd> getCumulativeBiVector(double t) const {
	      return Eigen::CwiseNullaryOp <BiVector, Eigen::VectorXd>(numValidTimeSegments(), 1, BiVector(segmentIndex(t), getLocalCumulativeBiVector(t), 1));
      }

      Eigen::MatrixXd segmentIntegral(int segmentIdx, const Eigen::MatrixXd& W, int derivativeOrder) const;
      
      Eigen::MatrixXd segmentQuadraticIntegral(const Eigen::MatrixXd & W, int segmentIdx, int derivativeOrder) const;
      Eigen::MatrixXd segmentQuadraticIntegralDiag(const Eigen::VectorXd & Wdiag, int segmentIdx, int derivativeOrder) const;
      Eigen::MatrixXd curveQuadraticIntegral(const Eigen::MatrixXd & W, int derivativeOrder) const;
      Eigen::MatrixXd curveQuadraticIntegralDiag(const Eigen::VectorXd & Wdiag, int derivativeOrder) const;
      
        
      sparse_block_matrix::SparseBlockMatrix<  Eigen::MatrixXd> curveQuadraticIntegralSparse(const  Eigen::MatrixXd & W, int derivativeOrder) const;
      sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> curveQuadraticIntegralDiagSparse(const Eigen::VectorXd & Wdiag, int derivativeOrder) const;
        
      void initConstantSpline(double t_min, double t_max, int numSegments, const Eigen::VectorXd & constant);
      
    private:
      /** 
       * An internal function to find the segment of the knot sequence
       * that the time t falls in. The function returns the 
       * value \f$ u = \frac{t - t_i}{t_{i+1} - t_i} \f$ and the index \f$i\f$
       * 
       * @param t The time being queried.
       * 
       * @return A pair with the first value \f$ u = \frac{t - t_i}{t_{i+1} - t_i} \f$ and the second value the index \f$i\f$
       */
      std::pair<double,int> computeUAndTIndex(double t) const;

      /** 
       * An internal function to find the segment of the knot sequence
       * that the time t falls in. The function returns the width of the 
       * knot segment \f$ \Delta t_i = t_{i+1} - t_i \f$ and the index \f$i\f$
       * 
       * @param t The time being queried.
       * 
       * @return A pair with the first value \f$ \Delta t_i = t_{i+1} - t_i \f$ and the second value the index \f$i\f$
       */
      std::pair<double,int> computeTIndex(double t) const;

      /** 
       * Compute the vector \f$ \mathbf u(t) \f$ for a spline of
       * order \f$ S \f$, this is an \f$ S \times 1 \f$ vector.
       * 
       * At derivative order 0 (no derivative), this vector is
       * \f$ \mathbf u(t) = \left [ 1 \; u(t) \; u(t)^2 \; \dots \; u(t)^{S-1} \right ]^T \f$
       *
       * For higher derivative order \f$ M \f$, the vector returned is 
       * \f$ \mathbf u^{(M)}(t) = \frac{\partial^{(M)}\mathbf u(t)}{\partial t^{(M)}}
       * 
       * @param uval the value \f$ u(t) \f$
       * @param segmentIndex 
       * @param derivativeOrder 
       * 
       * @return 
       */
      Eigen::VectorXd computeU(double uval, int segmentIndex, int derivativeOrder) const;
      
      int basisMatrixIndexFromStartingKnotIndex(int startingKnotIndex) const;      
      int startingKnotIndexFromBasisMatrixIndex(int basisMatrixIndex) const;      
      const Eigen::MatrixXd & basisMatrixFromKnotIndex(int knotIndex) const;


      /** 
       * Throws an exception if the knot sequence is not non-decreasing.
       * 
       * @param knots The knot sequence to verify.
       */
      void verifyKnotSequence(const std::vector<double> & knots);

      /** 
       * Initialize the basis matrices based on the current knot sequence.
       * There is one basis matrix for each valid time segment defined by the spline.
       * 
       * Implemented using the recursive basis matrix algorithm from
       * Qin, Kaihuai, General matrix representations for B-splines, The Visual Computer (2000) 16:177–186
       * 
       */
      void initializeBasisMatrices();

      /** 
       * The recursive function used to implement the recursive basis matrix algorithm from
       * Qin, Kaihuai, General matrix representations for B-splines, The Visual Computer (2000) 16:177–186
       * 
       * @param k The order of the matrix requested.
       * @param i The time segment of the basis matrix
       * 
       * @return 
       */
      Eigen::MatrixXd M(int k, int i);

      /** 
       * A helper function for producing the M matrices. Defined in
       * Qin, Kaihuai, General matrix representations for B-splines, The Visual Computer (2000) 16:177–186
       * 
       */
      double d_0(int k, int i, int j);

      /** 
       * A helper function for producing the M matrices. Defined in
       * Qin, Kaihuai, General matrix representations for B-splines, The Visual Computer (2000) 16:177–186
       * 
       */
      double d_1(int k, int i, int j);

      /// The order of the spline.
      int splineOrder_;

      /// The knot sequence used by the B-spline.
      std::vector<double> knots_;

      /// The coefficient matrix used by the B-Spline. Each column can be seen as a 
      /// single vector-valued spline coefficient.
      /// This is stored explicityl in column major order to ensure that each column (i.e.
      /// a single vector-valued spline coefficient) is stored in contiguous memory. This
      /// allows one to, for example, map a single spline coefficient using the Eigen::Map type.
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> coefficients_;
      //Eigen::MatrixXd coefficients_;

      /// The basis matrices for each time segment the B-spline is defined over.
      std::vector<Eigen::MatrixXd> basisMatrices_;
    };

} // namespace bsplines


#endif /* _BSPLINE_HPP */
