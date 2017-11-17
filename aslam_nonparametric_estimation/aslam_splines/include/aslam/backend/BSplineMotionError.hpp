#ifndef ASLAM_BACKEND_MOTION_ERROR_HPP
#define ASLAM_BACKEND_MOTION_ERROR_HPP

#include <aslam/backend/ErrorTerm.hpp>

#include <bsplines/BSplinePose.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>

namespace aslam {
  namespace backend {
    
    // An error term implementing our observation model.
    // This class derives from ErrorTermFs<1> because the
    // errors are of dimension 1.
    template<class SPLINE_T>
    class BSplineMotionError : public ErrorTermFs<1>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //  typedef aslam::splines::BSplinePoseDesignVariable spline_t;
      typedef SPLINE_T spline_t;
      typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> sbm_t;
        
        
      // BSplineMotionError(ScalarDesignVariable * x_k, ScalarDesignVariable * w, double y, double sigma2_n);
      BSplineMotionError(spline_t * splineDV, Eigen::MatrixXd W);
      BSplineMotionError(spline_t * splineDV, Eigen::MatrixXd W, unsigned int errorTermOrder);
        
      virtual ~BSplineMotionError();
        
    
      // usefull for debugging / error checking
      Eigen::MatrixXd Q() { return _Q.toDense(); };
      Eigen::VectorXd rhs();
        
        
    protected:
      /// This is the inteface required by ErrorTermFs<>
      
      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & J) const;

        virtual void buildHessianImplementation(SparseBlockMatrix & outHessian, Eigen::VectorXd & outRhs,bool useMEstimator);
              
    private:
   //   ScalarDesignVariable * _x_k;
   //   ScalarDesignVariable * _w;
        spline_t * _splineDV;
        Eigen::MatrixXd _W;
        sbm_t _Q;
        unsigned int _coefficientVectorLength;

        void initialize(spline_t * splineDV, Eigen::MatrixXd W, unsigned int errorTermOrder);

    };

  } // namespace backend
} // namespace aslam

#include "implementation/BSplineMotionError.hpp"


#endif /* ASLAM_BACKEND_MOTION_ERROR_HPP */
