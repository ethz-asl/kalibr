#ifndef ASLAM_PYTHON_EXPORT_DESIGN_VARIABLE_ADAPTER_HPP
#define ASLAM_PYTHON_EXPORT_DESIGN_VARIABLE_ADAPTER_HPP
#include <sstream>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/DesignVariableAdapter.hpp>
#include <sm/python/unique_register_ptr_to_python.hpp>


namespace aslam {
  namespace python {


  template<typename DESIGN_VARIABLE_T>
  void exportDesignVariableAdapter(const std::string & name)
  {

    using namespace boost::python;
    using namespace aslam;
    using namespace aslam::backend;
    using namespace aslam::cameras;
    typedef DESIGN_VARIABLE_T dv_t;

	//dv_t & (DesignVariableAdapter<dv_t>::*valueFnPtr)(void) = &DesignVariableAdapter<dv_t>::value;

	

  class_< DesignVariableAdapter<dv_t>, boost::shared_ptr<DesignVariableAdapter<dv_t> >, bases<DesignVariable> >( name.c_str(), init<boost::shared_ptr< dv_t> >() )
	.def("value", &DesignVariableAdapter<dv_t>::valuePtr)
  ;
  sm::python::unique_register_ptr_to_python<boost::shared_ptr<DesignVariableAdapter<dv_t> > >();
  //boost::python::implicitly_convertible<boost::shared_ptr<DesignVariableAdapter<dv_t> >, boost::shared_ptr<DesignVariable> >();
  
  }





  } // namespace python
} // namespace aslam


#endif /* ASLAM_PYTHON_EXPORT_DESIGN_VARIABLE_ADAPTER_HPP */
