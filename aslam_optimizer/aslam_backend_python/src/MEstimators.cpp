#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
using namespace boost::python;
using namespace aslam::backend;


void exportMEstimators()
{

  class_< MEstimator, boost::shared_ptr<MEstimator>, boost::noncopyable >("MEstimator", no_init )
    .def("getWeight", &MEstimator::getWeight)
    .def("name", &MEstimator::name)
    ;

  class_< NoMEstimator, boost::shared_ptr<NoMEstimator>, bases<MEstimator> >("NoMEstimator", init<>())
  ;

  class_< GemanMcClureMEstimator, boost::shared_ptr<GemanMcClureMEstimator>, bases<MEstimator> >("GemanMcClureMEstimator", init<double>())
  ;

    class_< CauchyMEstimator, boost::shared_ptr<CauchyMEstimator>, bases<MEstimator> >("CauchyMEstimator", init<double>())
  ;

  
  
  class_< HuberMEstimator, boost::shared_ptr<HuberMEstimator>, bases<MEstimator> >("HuberMEstimator", init<double>())
  ;

  class_< BlakeZissermanMEstimator, boost::shared_ptr<BlakeZissermanMEstimator>, bases<MEstimator> >("BlakeZissermanMEstimator", init<double>("BlakeZissermanMEstimator( dimensionOfErrorTerm)"))
      .def(init<double,double,double>("BlakeZissermanMEstimator(dim, pCut, wCut)\n* @param df    The dimension of your error term\n* @param pCut  The probability of error that you want to down-weight. 0.99 means\n*              that the 99% most likely squared errors will not be much down-weighted\n*              and the remaining 1% will be smoothly cut to zero weight.\n* @param wCut  The weight (between 0 and 1 not inclusive) that you want the above probability to have."))


  ;



}
