#include <aslam/backend/SimpleSplineError.hpp>
#include <stdio.h>
namespace aslam {
    namespace backend {
        

    
        template<class SPLINE_T>
        SimpleSplineError<SPLINE_T>::SimpleSplineError(spline_t* splineDV, expression_t* splineExpression, Eigen::Matrix<double, spline_t::Dimension,1> y, double t):
        _splineDV(splineDV), _splineExpression(splineExpression), _y(y), _t(t)
        {
            
            // Add the design variables to the error term:       
            DesignVariable::set_t dvV;
        	//for ( unsigned int i = 0; i < _splineExpression->numDesignVariables(); i++) {
                //	dvV.push_back(_splineExpression->designVariable(i));
                //}
        	_splineExpression->getDesignVariables(dvV);
            setDesignVariablesIterator(dvV.begin(), dvV.end());
            
        }


        template<class SPLINE_T>
        SimpleSplineError<SPLINE_T>::~SimpleSplineError()
        {

            
            
        }


        /// \brief evaluate the error term and return the weighted squared error e^T invR e
        template<class SPLINE_T>
        double SimpleSplineError<SPLINE_T>::evaluateErrorImplementation()
        {
            Eigen::VectorXd error = (_splineExpression->toValue() - _y);
            parent_t::setError(error);
            return error.transpose()*error;
            
        }


        /// \brief evaluate the jacobians
        template<class SPLINE_T>
        void SimpleSplineError<SPLINE_T>::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians) const
        {
            
            _splineExpression->evaluateJacobians(_jacobians);

            
            
        }
          
          
        
          

    } // namespace backend
} // namespace aslam
