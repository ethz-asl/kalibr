#ifndef SM_PYTHON_ID_HPP
#define SM_PYTHON_ID_HPP

#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/to_python_converter.hpp>
#include <Python.h>
#include <boost/cstdint.hpp>

namespace sm { namespace python {

        // to use:     sm::python::Id_python_converter<FrameId>::register_converter();


    // adapted from http://misspent.wordpress.com/2009/09/27/how-to-write-boost-python-converters/
    
    template<typename ID_T>
    struct Id_python_converter
    {
      typedef ID_T id_t;

      // The "Convert from C to Python" API
      static PyObject * convert(id_t const & id){
	PyObject * i = PyInt_FromLong(id.getId());
    // It seems that the call to "incref(.)" caused a memory leak!
    // I will check this in hoping it doesn't cause any instability.
	return i;//boost::python::incref(i);
      }
      
      // The "Convert from Python to C" API
      // Two functions: convertible() and construct()
      static void * convertible(PyObject* obj_ptr)
      {
          if (!(PyInt_Check(obj_ptr) || PyLong_Check(obj_ptr)))
              return 0;
          
          return obj_ptr;
      }

      static void construct(
			    PyObject* obj_ptr,
			    boost::python::converter::rvalue_from_python_stage1_data* data)
      {

	// Get the value.
          boost::uint64_t value;
          if ( PyInt_Check(obj_ptr) ) {
              value = PyInt_AsUnsignedLongLongMask(obj_ptr);
          } else {
              value = PyLong_AsUnsignedLongLong(obj_ptr);
          }
        
          void* storage = ((boost::python::converter::rvalue_from_python_storage<id_t>*)
                           data)->storage.bytes;
          
          
          new (storage) id_t(value);
          
          // Stash the memory chunk pointer for later use by boost.python
          data->convertible = storage;
      }
  



      // The registration function.
      static void register_converter()
      {
         
          // This code checks if the type has already been registered.
          // http://stackoverflow.com/questions/9888289/checking-whether-a-converter-has-already-been-registered
          boost::python::type_info info = boost::python::type_id<id_t>(); 
          const boost::python::converter::registration* reg = boost::python::converter::registry::query(info); 
          if (reg == NULL || reg->m_to_python == NULL) {
              // This is the code to register the type.
              boost::python::to_python_converter<id_t,Id_python_converter>();
              boost::python::converter::registry::push_back(
                  &convertible,
                  &construct,
                  boost::python::type_id<id_t>());

          }


      }
  
    };

  }}

#endif /* SM_PYTHON_ID_HPP */
