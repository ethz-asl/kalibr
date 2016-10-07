// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/shared_ptr.hpp>

#include <aslam/backend/ScalarDesignVariable.hpp>
#include <aslam/backend/ErrorTermObservation.hpp>
#include <aslam/backend/ErrorTermMotion.hpp>
#include <aslam/backend/ErrorTermPrior.hpp>

// The title of this library must match exactly the title of the produced library
BOOST_PYTHON_MODULE(libaslam_backend_tutorial_python)
{
  // Here we have to export the design variables and error terms used in this problem.
  // The aslam_python library exports the interface of the base classes so we
  // leverage this work by pointing the boost::python wrapper at these base classes.

  using namespace boost::python;
  using namespace aslam::backend;

  class_<
    // The first template argument is the class we are wrapping.
    ScalarDesignVariable, 
    // This tells boost::python so wrap this class as a boost shared
    // pointer. While this is not strictly required, it does help as
    // it will allow us to pass objects from python to the OptimizationProblem
    // as boost shared pointers.
    boost::shared_ptr<ScalarDesignVariable>, 
    // This template argument tells boost::python to inherit the interface of
    // DesignVariable.
    bases<DesignVariable>
    >("ScalarDesignVariable", // <-- this sets the name of the class in Python
      init<double>("ScalarDesignVariable( initialValue )")) // <-- Here we tell python that the default constructor takes a double. The string is for documentation.
    .def("value", &ScalarDesignVariable::value) // <-- And here we export methods specific to this class.
    ;


class_<ErrorTermObservation, boost::shared_ptr<ErrorTermObservation>, bases<ErrorTerm> >
("ErrorTermObservation", 
 init<ScalarDesignVariable *,ScalarDesignVariable *,double,double>("ErrorTermObservation(ScalarDesignVariable x_k, ScalarDesignVariable w, double y, double sigma2_n)"))
;

class_<ErrorTermMotion, boost::shared_ptr<ErrorTermMotion>, bases<ErrorTerm> >
("ErrorTermMotion", 
 init<ScalarDesignVariable *,ScalarDesignVariable *,double,double>("ErrorTermMotion(ScalarDesignVariable x_k, ScalarDesignVariable x_kp1, double u, double sigma2_u)"))
;

class_<ErrorTermPrior, boost::shared_ptr<ErrorTermPrior>, bases<ErrorTerm> >
("ErrorTermPrior", 
 init<ScalarDesignVariable *,double,double>("ErrorTermPrior(ScalarDesignVariable * x, double hat_x, double sigma2_x)"))
;	 
   
}
