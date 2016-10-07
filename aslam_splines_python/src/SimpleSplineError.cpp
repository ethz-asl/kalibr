#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/splines/BSplineDesignVariable.hpp>
#include <aslam/backend/SimpleSplineError.hpp>

#include <boost/shared_ptr.hpp>



using namespace boost::python;
using namespace aslam::splines;
using namespace aslam::backend;

void exportSimpleSplineError()
{


    class_<SimpleSplineError<BSplineDesignVariable<1> >, boost::shared_ptr<SimpleSplineError<BSplineDesignVariable<1> > >, bases<ErrorTerm> >
    ("SimpleSplineError1", init<BSplineDesignVariable<1>*, VectorExpression<1>*, Eigen::Matrix<double,BSplineDesignVariable<1>::Dimension, 1> , double >("SimpleSplineError(BSplineDesignVariable<1>, BSPlineExpression<1>,Values, Time )"))

          ;
    // .def(init<aslam::splines::BSplineDesignVariable*, aslam::backend::VectorExpression<1>*, Eigen::VectorXd, double >("SimpleSplineError(BSplineDesignVariable, BSPlineExpression, )"))
    

  //  ;
    
    
}
