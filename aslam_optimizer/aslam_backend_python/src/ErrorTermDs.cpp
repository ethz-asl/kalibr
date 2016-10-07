#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTermDs.hpp>
#include <boost/shared_ptr.hpp>
using namespace boost::python;
using namespace aslam::backend;


void exportErrorTermDs()
{
  
  class_<ErrorTermDs, boost::shared_ptr<ErrorTermDs>, bases<ErrorTerm>, boost::noncopyable>("ErrorTermDs", no_init)
    /// \brief evaluate the error term.
    ///        After this is called, the _squaredError is filled in with \f$ \mathbf e^T \mathbf R^{-1} \mathbf e \f$
;

  
  ;

}
