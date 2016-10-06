/**
 * @file   boost_python_headers.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Mon Dec 12 10:36:03 2011
 * 
 * @brief  A header that specializes boost-python to work with fixed-sized Eigen types.
 * 
 * The original version of this library did not include these specializations and this caused
 * assert failures when running on Ubuntu 10.04 32-bit. More information about fixed-size 
 * vectorizable types in Eigen is available here: 
 * http://eigen.tuxfamily.org/dox-devel/TopicFixedSizeVectorizable.html
 *
 * This code has been tested on Ubunutu 10.04 64 and 32 bit, OSX Snow Leopard and OSX Lion.
 *
 * This code is derived from boost/python/converter/arg_from_python.hpp
 * Copyright David Abrahams 2002.
 * Distributed under the Boost Software License, Version 1.0. (See http://www.boost.org/LICENSE_1_0.txt)
 * 
 */
#ifndef NUMPY_EIGEN_CONVERTERS_HPP
#define NUMPY_EIGEN_CONVERTERS_HPP

#include <Eigen/Core>
#include <boost/python.hpp>
#include <boost/python/detail/referent_storage.hpp>
#include <boost/python/converter/arg_from_python.hpp>
#include <boost/python/converter/rvalue_from_python_data.hpp>
#include <boost/python/tuple.hpp>


namespace boost { namespace python { namespace detail {
      template<typename T>
      struct referent_size;

      // This bit of code makes sure we have 16 extra bytes to do the pointer alignment for fixed-sized Eigen types
       template<typename T, int A, int B, int C, int D, int E>
       struct referent_size< Eigen::Matrix<T,A,B,C,D,E>& >
       {
       	// Add 16 bytes so we can get alignment
       	BOOST_STATIC_CONSTANT( std::size_t, value = sizeof(Eigen::Matrix<T,A,B,C,D,E>) + 16);
       };

      // This bit of code makes sure we have 16 extra bytes to do the pointer alignment for fixed-sized Eigen types
       template<typename T, int A, int B, int C, int D, int E>
       struct referent_size< Eigen::Matrix<T,A,B,C,D,E> const & >
       {
       	// Add 16 bytes so we can get alignment
       	BOOST_STATIC_CONSTANT( std::size_t, value = sizeof(Eigen::Matrix<T,A,B,C,D,E>) + 16);
       };

      // This bit of code makes sure we have 16 extra bytes to do the pointer alignment for fixed-sized Eigen types
       template<typename T, int A, int B, int C, int D, int E>
       struct referent_size< Eigen::Matrix<T,A,B,C,D,E> >
       {
       	// Add 16 bytes so we can get alignment
       	BOOST_STATIC_CONSTANT( std::size_t, value = sizeof(Eigen::Matrix<T,A,B,C,D,E>) + 16);
       };


    }}}

namespace boost { namespace python { namespace converter { 


      template<typename S, int A, int B, int C, int D, int E>
      struct rvalue_from_python_data< Eigen::Matrix<S,A,B,C,D,E> const &> : rvalue_from_python_storage< Eigen::Matrix<S,A,B,C,D,E> const & >
      {
	typedef typename Eigen::Matrix<S,A,B,C,D,E> T;
# if (!defined(__MWERKS__) || __MWERKS__ >= 0x3000)		 \
  && (!defined(__EDG_VERSION__) || __EDG_VERSION__ >= 245)	 \
  && (!defined(__DECCXX_VER) || __DECCXX_VER > 60590014)		\
  && !defined(BOOST_PYTHON_SYNOPSIS) /* Synopsis' OpenCXX has trouble parsing this */
	// This must always be a POD struct with m_data its first member.
	BOOST_STATIC_ASSERT(BOOST_PYTHON_OFFSETOF(rvalue_from_python_storage<T>,stage1) == 0);
# endif

	// The usual constructor 
	rvalue_from_python_data(rvalue_from_python_stage1_data const & _stage1)
	{
	  this->stage1 = _stage1;
	}


	// This constructor just sets m_convertible -- used by
	// implicitly_convertible<> to perform the final step of the
	// conversion, where the construct() function is already known.
	rvalue_from_python_data(void* convertible)
	{
	  this->stage1.convertible = convertible;
	}
	
	// Destroys any object constructed in the storage.
	~rvalue_from_python_data()
	{
	  // Realign the pointer and destroy
	  if (this->stage1.convertible == this->storage.bytes)
	    {
	      void * storage = reinterpret_cast<void *>(this->storage.bytes);
	      T * aligned = reinterpret_cast<T *>(reinterpret_cast<void *>((reinterpret_cast<size_t>(storage) & ~(size_t(15))) + 16));
	      
	      //std::cout << "Destroying " << (void*)aligned << std::endl;
	      aligned->T::~T();
	    }
	}
      private:
	typedef typename add_reference<typename add_cv<T>::type>::type ref_type;
      };


      


      // Used when T is a plain value (non-pointer, non-reference) type or
      // a (non-volatile) const reference to a plain value type.
      template<typename S, int A, int B, int C, int D, int E>
      struct arg_rvalue_from_python< Eigen::Matrix<S,A,B,C,D,E> >
      {
	typedef Eigen::Matrix<S,A,B,C,D,E> const & T;
	typedef typename boost::add_reference<
	  T
	  // We can't add_const here, or it would be impossible to pass
	  // auto_ptr<U> args from Python to C++
	  >::type result_type;
	
	arg_rvalue_from_python(PyObject * obj) : m_data(converter::rvalue_from_python_stage1(obj, registered<T>::converters))
					       , m_source(obj)
	{
	  
	}
	bool convertible() const
	{
	  return m_data.stage1.convertible != 0;
	}

	
# if BOOST_MSVC < 1301 || _MSC_FULL_VER > 13102196
	typename arg_rvalue_from_python<T>::
# endif 
	result_type operator()()
	{
	  if (m_data.stage1.construct != 0)
	    m_data.stage1.construct(m_source, &m_data.stage1);
	  
	  // Here is the magic...
	  // Realign the pointer
	  void * storage = reinterpret_cast<void *>(m_data.storage.bytes);
	  void * aligned = reinterpret_cast<void*>((reinterpret_cast<size_t>(storage) & ~(size_t(15))) + 16);
	  
	  return python::detail::void_ptr_to_reference(aligned, (result_type(*)())0);
	}

      private:
	rvalue_from_python_data<result_type> m_data;
	PyObject* m_source;
      };


      // Used when T is a plain value (non-pointer, non-reference) type or
      // a (non-volatile) const reference to a plain value type.
      template<typename S, int A, int B, int C, int D, int E>
      struct arg_rvalue_from_python< Eigen::Matrix<S,A,B,C,D,E> const & >
      {
	typedef Eigen::Matrix<S,A,B,C,D,E> const & T;
	typedef typename boost::add_reference<
	  T
	  // We can't add_const here, or it would be impossible to pass
	  // auto_ptr<U> args from Python to C++
	  >::type result_type;
	
	arg_rvalue_from_python(PyObject * obj) : m_data(converter::rvalue_from_python_stage1(obj, registered<T>::converters))
					       , m_source(obj)
	{
	  
	}
	bool convertible() const
	{
	  return m_data.stage1.convertible != 0;
	}

	
# if BOOST_MSVC < 1301 || _MSC_FULL_VER > 13102196
	typename arg_rvalue_from_python<T>::
# endif 
	result_type operator()()
	{
	  if (m_data.stage1.construct != 0)
	    m_data.stage1.construct(m_source, &m_data.stage1);
	  
	  // Here is the magic...
	  // Realign the pointer
	  void * storage = reinterpret_cast<void *>(m_data.storage.bytes);
	  void * aligned = reinterpret_cast<void*>((reinterpret_cast<size_t>(storage) & ~(size_t(15))) + 16);
	  
	  return python::detail::void_ptr_to_reference(aligned, (result_type(*)())0);
	}

      private:
	rvalue_from_python_data<result_type> m_data;
	PyObject* m_source;
      };

      // Used when T is a plain value (non-pointer, non-reference) type or
      // a (non-volatile) const reference to a plain value type.
      template<typename S, int A, int B, int C, int D, int E>
      struct arg_rvalue_from_python< Eigen::Matrix<S,A,B,C,D,E> const >
      {
	typedef Eigen::Matrix<S,A,B,C,D,E> const & T;
	typedef typename boost::add_reference<
	  T
	  // We can't add_const here, or it would be impossible to pass
	  // auto_ptr<U> args from Python to C++
	  >::type result_type;
	
	arg_rvalue_from_python(PyObject * obj) : m_data(converter::rvalue_from_python_stage1(obj, registered<T>::converters))
					       , m_source(obj)
	{
	  
	}
	bool convertible() const
	{
	  return m_data.stage1.convertible != 0;
	}

	
# if BOOST_MSVC < 1301 || _MSC_FULL_VER > 13102196
	typename arg_rvalue_from_python<T>::
# endif 
	result_type operator()()
	{
	  if (m_data.stage1.construct != 0)
	    m_data.stage1.construct(m_source, &m_data.stage1);
	  
	  // Here is the magic...
	  // Realign the pointer
	  void * storage = reinterpret_cast<void *>(m_data.storage.bytes);
	  void * aligned = reinterpret_cast<void*>((reinterpret_cast<size_t>(storage) & ~(size_t(15))) + 16);
	  
	  return python::detail::void_ptr_to_reference(aligned, (result_type(*)())0);
	}

      private:
	rvalue_from_python_data<result_type> m_data;
	PyObject* m_source;
      };


    }}}


#endif /* NUMPY_EIGEN_CONVERTERS_HPP */
