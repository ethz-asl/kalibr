#ifndef ASLAM_BACKEND_SPLINE_ERROR_HPP
#define ASLAM_BACKEND_SPLINE_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>

#include <bsplines/BSplinePose.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>

namespace aslam {
  namespace backend {
    
    // An error term implementing our observation model.
    // This class derives from ErrorTermFs<1> because the
    // errors are of dimension 1.
    template<class SPLINE_T>
    class SimpleSplineError : public ErrorTermFs<1>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //  typedef aslam::splines::BSplinePoseDesignVariable spline_t;
      typedef SPLINE_T spline_t;
      typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> sbm_t;
      typedef ErrorTermFs< 1 > parent_t;
      typedef aslam::backend::VectorExpression<spline_t::Dimension> expression_t;
        

      SimpleSplineError(spline_t * splineDV, expression_t* splineExpression, Eigen::Matrix<double, spline_t::Dimension,1> y, double t);
        
      virtual ~SimpleSplineError();
        
        
    protected:
      /// This is the inteface required by ErrorTermFs<>
      
      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & J) const;

      //  virtual void buildHessianImplementation(SparseBlockMatrix & outHessian, Eigen::VectorXd & outRhs,bool useMEstimator);
      
     // virtual const JacobianContainer & getJacobiansImplementation() const;
        
    private:

        spline_t * _splineDV;
       // boost::shared_ptr< expression_t > _splineExpression;
        expression_t* _splineExpression;
        
        Eigen::VectorXd _y;
        double _t;

    };

  } // namespace backend
} // namespace aslam

#include "implementation/SimpleSplineError.hpp"


#endif /* ASLAM_BACKEND_SPLINE_ERROR_HPP */
