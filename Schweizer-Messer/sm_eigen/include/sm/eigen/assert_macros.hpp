/**
 * @file   assert_macros.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Mon Dec 12 11:53:43 2011
 * 
 * @brief  Code for checking if Eigen matrices are finite. It is slow. Use sparingly.
 * 
 * 
 */


#ifndef SM_EIGEN_ISNAN_HPP
#define SM_EIGEN_ISNAN_HPP

#include <cmath>
#include <sm/assert_macros.hpp>


#define SM_ASSERT_MAT_IS_FINITE(exceptionType, matrix, message)		\
  {									\
  for(int r = 0; r < matrix.rows(); ++r)				\
    {									\
      for(int c = 0; c < matrix.cols(); ++c)				\
	{								\
	  if(!std::isfinite(matrix(r,c)))				\
	    {								\
	      std::stringstream sm_assert_stringstream;		\
	      sm_assert_stringstream << "debug assert( isfinite(" << #matrix << "(" << r << ", " << c << ") ) failed. [ isfinite(" << matrix(r,c) << " ) ]" << message << std::endl << matrix; \
	      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
	    }								\
	}								\
     }   								\
}


#ifndef NDEBUG

#define SM_ASSERT_MAT_IS_FINITE_DBG(exceptionType, matrix, message)		\
  {									\
  for(int r = 0; r < matrix.rows(); ++r)				\
    {									\
      for(int c = 0; c < matrix.cols(); ++c)				\
	{								\
	  if(!std::isfinite(matrix(r,c)))				\
	    {								\
	      std::stringstream sm_assert_stringstream;		\
	      sm_assert_stringstream << "assert( isfinite(" << #matrix << "(" << r << ", " << c << ") ) failed. [ isfinite(" << matrix(r,c) << " ) ]" << message << std::endl << matrix; \
	      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
	    }								\
	}								\
     }   								\
}


#else

#define SM_ASSERT_MAT_IS_FINITE_DBG(exceptionType, matrix, message)

#endif


#endif /* SM_EIGEN_ISNAN_HPP */
