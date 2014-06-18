/**
 * @file   assert_macros.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Mon Dec 12 11:22:20 2011
 * 
 * @brief  Assert macros to facilitate rapid prototyping. Use them early and often.
 * 
 * 
 */

#ifndef SM_ASSERT_HPP
#define SM_ASSERT_HPP

#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include "source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define SM_DEFINE_EXCEPTION(exceptionName, exceptionParent)				\
  class exceptionName : public exceptionParent {						\
  public:																\
  exceptionName(const char * message) : exceptionParent(message) {}		\
  exceptionName(std::string const & message) : exceptionParent(message) {} \
  virtual ~exceptionName() throw() {}									\
  };									  

namespace sm {

  namespace detail {

    template<typename SM_EXCEPTION_T>
    inline void sm_throw_exception(std::string const & exceptionType, sm::source_file_pos sfp, std::string const & message)
    {
      std::stringstream sm_assert_stringstream;
#ifdef _WIN32
      // I have no idea what broke this on Windows but it doesn't work with the << operator.
      sm_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#else
      sm_assert_stringstream << exceptionType <<  sfp << " " << message;
#endif
      throw(SM_EXCEPTION_T(sm_assert_stringstream.str()));
    }

    template<typename SM_EXCEPTION_T>
    inline void sm_throw_exception(std::string const & exceptionType, std::string const & function, std::string const & file,
								   int line, std::string const & message)
    {
      sm_throw_exception<SM_EXCEPTION_T>(exceptionType, sm::source_file_pos(function,file,line),message);
    }


  } // namespace sm::detail

  template<typename SM_EXCEPTION_T>
  inline void sm_assert_throw(bool assert_condition, std::string message, sm::source_file_pos sfp) {
    if(!assert_condition)
      {
		detail::sm_throw_exception<SM_EXCEPTION_T>("", sfp,message);
      }
  }



} // namespace sm



#define SM_THROW(exceptionType, message) {								\
    std::stringstream sm_assert_stringstream;							\
    sm_assert_stringstream << message;									\
    sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, sm_assert_stringstream.str()); \
  }


#define SM_THROW_SFP(exceptionType, SourceFilePos, message){			\
    std::stringstream sm_assert_stringstream;							\
    sm_assert_stringstream << message;									\
    sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, sm_assert_stringstream.str()); \
  }

#define SM_ASSERT_TRUE(exceptionType, condition, message)				\
  if(!(condition))														\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_FALSE(exceptionType, condition, message)				\
  if((condition))														\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))							\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_LT(exceptionType, value, upperBound, message)			\
  if((value) >= (upperBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_GE(exceptionType, value, lowerBound, message)			\
  if((value) < (lowerBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_LE(exceptionType, value, upperBound, message)			\
  if((value) > (upperBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_GT(exceptionType, value, lowerBound, message)			\
  if((value) <= (lowerBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_EQ(exceptionType, value, testValue, message)			\
  if((value) != (testValue))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_NE(exceptionType, value, testValue, message)			\
  if((value) == (testValue))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))						\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#ifndef NDEBUG

#define SM_THROW_DBG(exceptionType, message){							\
    std::stringstream sm_assert_stringstream;							\
    sm_assert_stringstream << message;									\
    sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, sm_assert_stringstream.str()); \
  }



#define SM_ASSERT_TRUE_DBG(exceptionType, condition, message)			\
  if(!(condition))														\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_FALSE_DBG(exceptionType, condition, message)			\
  if((condition))														\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, sm_assert_stringstream.str()); \
    }


#define SM_ASSERT_DBG_RE( condition, message) SM_ASSERT_DBG(std::runtime_error, condition, message)

#define SM_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))							\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_LT_DBG(exceptionType, value, upperBound, message)		\
  if((value) >= (upperBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)		\
  if((value) < (lowerBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_LE_DBG(exceptionType, value, upperBound, message)		\
  if((value) > (upperBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }

#define SM_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)		\
  if((value) <= (lowerBound))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_EQ_DBG(exceptionType, value, testValue, message)		\
  if((value) != (testValue))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }


#define SM_ASSERT_NE_DBG(exceptionType, value, testValue, message)		\
  if((value) == (testValue))												\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }



#define SM_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))						\
    {																	\
      std::stringstream sm_assert_stringstream;							\
      sm_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      sm::detail::sm_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,sm_assert_stringstream.str()); \
    }


#define SM_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define SM_OUT(X)
#define SM_THROW_DBG(exceptionType, message)
#define SM_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define SM_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define SM_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define SM_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define SM_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define SM_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define SM_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define SM_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define SM_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define SM_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)	
#endif



#endif
