#ifndef SM_PYTHON_UNIQUE_REGISTER_PTR_TO_PYTHON
#define SM_PYTHON_UNIQUE_REGISTER_PTR_TO_PYTHON

#include <boost/python/register_ptr_to_python.hpp>
#include <boost/python/type_id.hpp>

namespace sm {
namespace python {

// Calls register_ptr_to_python<D>() in case the class has not already been
// registered. Related to
// https://stackoverflow.com/questions/9888289/checking-whether-a-converter-has-already-been-registered#
template <typename D>
void unique_register_ptr_to_python() {
  boost::python::type_info info = boost::python::type_id<D>(); 
  const boost::python::converter::registration* reg = boost::python::converter::registry::query(info);
  if (reg == NULL || reg->m_to_python == NULL) {
    boost::python::register_ptr_to_python<D>();
  }
}

}  // namespace python
}  // namespace sm

#endif  // SM_PYTHON_UNIQUE_REGISTER_PTR_TO_PYTHON

