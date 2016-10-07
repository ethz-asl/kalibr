#ifndef ASLAM_PYTHON_EXPORT_APRIORI_INFORMATION_ERROR_HPP
#define ASLAM_PYTHON_EXPORT_APRIORI_INFORMATION_ERROR_HPP
#include <sstream>
#include <aslam/backend/APrioriInformationError.hpp>


namespace aslam {
  namespace python {


  template<typename DESIGN_VARIABLE_T>
  void exportAPrioriInformationError(const std::string & name)
  {

    using namespace boost::python;
    using namespace aslam;
    using namespace aslam::backend;
    typedef DESIGN_VARIABLE_T dv_t;
	

    // add the corresponding a priori information error
    class_<APrioriInformationError<dv_t>, boost::shared_ptr<APrioriInformationError<dv_t> >, bases<ErrorTerm> >
            (name.c_str(), init<dv_t*, Eigen::MatrixXd >("APrioriInformationError(DesignVariable, invM)"))
             ;

  }





  } // namespace python
} // namespace aslam


#endif /* ASLAM_PYTHON_EXPORT_APRIORI_INFORMATION_ERROR_HPP */
