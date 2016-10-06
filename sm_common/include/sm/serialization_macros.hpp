/**
 * @file   serialization_macros.hpp
 * @author Simon Lynen <simon.lynen@mavt.ethz.ch>
 * @date   Thu May 22 02:55:13 2013
 *
 * @brief  Comparison macros to facilitate checking in serialization methods.
 *
 *
 */

#ifndef SM_SERIALIZATION_MACROS_HPP
#define SM_SERIALIZATION_MACROS_HPP

#include <type_traits>
#include <sstream>
#include <ostream>
#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/shared_ptr.hpp>
#include <sm/typetraits.hpp>

namespace cv {
class Mat;
}  // namespace cv

namespace sm_serialization {
namespace internal_types {
typedef char yes;
typedef int no;
}  // namespace internal_types
}  // namespace sm_serialization

struct AnyT {
  template<class T> AnyT(const T &);
};

sm_serialization::internal_types::no operator <<(const AnyT &, const AnyT &);

namespace sm{
namespace serialization{
namespace internal{

template<class T> sm_serialization::internal_types::yes check(const T&);
sm_serialization::internal_types::no check(sm_serialization::internal_types::no);

struct makeCompilerSilent{ //rm warning about unused functions
  void foo(){
    check(sm_serialization::internal_types::no());
    AnyT t(5);
    operator <<(t, t);
  }
};

//this template metaprogramming struct can tell us if there is the operator<< defined somewhere
template<typename StreamType, typename T>
class HasOStreamOperator {
  static StreamType & stream;
  static T & x;
 public:
  enum {
    value = sizeof(check(stream << x)) == sizeof(sm_serialization::internal_types::yes)
  };
};

template<typename StreamType, typename T>
class HasOStreamOperator<StreamType, boost::shared_ptr<T> > {
 public:
  enum {
     value = HasOStreamOperator<StreamType, T>::value
   };
};

template<typename StreamType, typename T>
class HasOStreamOperator<StreamType, T* > {
 public:
  enum {
     value = HasOStreamOperator<StreamType, T>::value
   };
};

template<typename StreamType, typename T>
class HasOStreamOperator<StreamType, T& > {
 public:
  enum {
     value = HasOStreamOperator<StreamType, T>::value
   };
};

//this template metaprogramming struct can tell us whether a class has a member
//function isBinaryEqual
template<typename T>
class HasIsBinaryEqual {
  //for non const methods (which is actually wrong)
  template<typename U, bool (U::*)(const T&)> struct Check;
  template<typename U> static char func(Check<U, &U::isBinaryEqual> *);
  template<typename U> static int func(...);
  //for const methods
  template<typename U, bool (U::*)(const T&) const> struct CheckConst;
  template<typename U> static char funcconst(CheckConst<U, &U::isBinaryEqual> *);
  template<typename U> static int funcconst(...);
 public:
  enum {
      value = (int)((sizeof(func<T>(0)) == sizeof(char)) || (sizeof(funcconst<T>(0)) == sizeof(char)))
  };
};
template<typename T>
class HasIsBinaryEqual<boost::shared_ptr<T> > {
 public:
  enum {
    value = HasIsBinaryEqual<T>::value
  };
};
template<typename T>
class HasIsBinaryEqual<T*> {
 public:
  enum {
    value = HasIsBinaryEqual<T>::value
  };
};
template<typename T>
class HasIsBinaryEqual<T&> {
 public:
  enum {
    value = HasIsBinaryEqual<T>::value
  };
};

//these structs are used to choose between isBinaryEqual function call and the
//default operator==
template<bool, typename A>
struct isSame;

template<typename A>
struct isSame<true, A> {
  static bool eval(const A& lhs, const A& rhs) {
    return lhs.isBinaryEqual(rhs);
  }
};

template<typename A>
struct isSame<true, boost::shared_ptr<A> > {
  static bool eval(const boost::shared_ptr<A>& lhs, const boost::shared_ptr<A>& rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return lhs->isBinaryEqual(*rhs);
  }
};

template<typename A>
struct isSame<true, A* > {
  static bool eval(const A* const lhs, const A* const rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return lhs->isBinaryEqual(*rhs);
  }
};

template<typename A>
struct isSame<false, A> {
  static bool eval(const A& lhs, const A& rhs) {
    return lhs == rhs;
  }
};

template<typename A>
struct isSame<false, A*> {
  static bool eval(A const * lhs, A const * rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return *lhs == *rhs;
  }
};

template<typename A>
struct isSame<false, boost::shared_ptr<A> > {
  static bool eval(const boost::shared_ptr<A>& lhs, const boost::shared_ptr<A>& rhs) {
    if (!lhs && !rhs) {
      return true;
    }
    if (!lhs || !rhs) {
      return false;
    }
    return *lhs == *rhs;
  }
};

//for opencv Mat we have to use the sm opencv isBinaryEqual method otherwise sm_common has to depend on sm_opencv

template<typename T, bool B>
struct checkTypeIsNotOpencvMat {
  enum{
    value = true,
  };
};

template<bool B>
struct checkTypeIsNotOpencvMat<cv::Mat, B> {
  enum{
    value = true, //yes true is correct here
  };
  BOOST_STATIC_ASSERT_MSG((B == !B) /*false*/, "You cannot use the macro SM_CHECKSAME or SM_CHECKSAMEMEMBER on opencv mat. Use sm::opencv::isBinaryEqual instead");
};

template<bool B>
struct checkTypeIsNotOpencvMat<boost::shared_ptr<cv::Mat>, B > {
  enum{
    value = true, //yes true is correct here
  };
  BOOST_STATIC_ASSERT_MSG((B == !B) /*false*/, "You cannot use the macro SM_CHECKSAME or SM_CHECKSAMEMEMBER on opencv mat. Use sm::opencv::isBinaryEqual instead");
};

template<bool B>
struct checkTypeIsNotOpencvMat<cv::Mat*, B> {
  enum{
    value = true, //yes true is correct here
  };
  BOOST_STATIC_ASSERT_MSG((B == !B) /*false*/, "You cannot use the macro SM_CHECKSAME or SM_CHECKSAMEMEMBER on opencv mat. Use sm::opencv::isBinaryEqual instead");
};

//if the object supports it stream to ostream, otherwise put NA
template<bool, typename A>
struct streamIf;

template<typename A>
struct streamIf<true, A> {
  static std::string eval(const A& rhs) {
    std::stringstream ss;
    ss << rhs;
    return ss.str();
  }
};

template<typename A>
struct streamIf<true, A*> {
  static std::string eval(const A* rhs) {
    if (!rhs) {
      return "NULL";
    }
    std::stringstream ss;
    ss << *rhs;
    return ss.str();
  }
};

template<typename A>
struct streamIf<true, boost::shared_ptr<A> > {
  static std::string eval(const boost::shared_ptr<A>& rhs) {
    if (!rhs) {
      return "NULL";
    }
    std::stringstream ss;
    ss << *rhs;
    return ss.str();
  }
};

template<typename A>
struct streamIf<true, boost::shared_ptr<const A> > {
  static std::string eval(const boost::shared_ptr<const A>& rhs) {
    if (!rhs) {
      return "NULL";
    }
    std::stringstream ss;
    ss << *rhs;
    return ss.str();
  }
};

template<typename A>
struct streamIf<false, A> {
  static std::string eval(const A&) {
    return "NA";
  }
};

struct VerboseChecker {
  static bool SetVerbose(bool verbose) {
    Instance().verbose_ = verbose;
    return true;  // Intended.
  }
  static bool Verbose() {
    return Instance().verbose_;
  }
 private:
  static VerboseChecker& Instance() {
    static VerboseChecker instance;
    return instance;
  }
  VerboseChecker() : verbose_(false) { }
  VerboseChecker(const VerboseChecker&);
  VerboseChecker& operator==(const VerboseChecker&);
  bool verbose_;
};
}  // namespace internal
}  // namespace serialization
}  // namespace sm

#define IS_CHECKSAME_CURRENTLY_VERBOSE \
  (sm::serialization::internal::VerboseChecker::Verbose())

#define SET_CHECKSAME_VERBOSITY(verbose) \
    sm::serialization::internal::VerboseChecker::SetVerbose(verbose)

#define SET_CHECKSAME_VERBOSE \
    SET_CHECKSAME_VERBOSITY(true)

#define SET_CHECKSAME_SILENT \
    SET_CHECKSAME_VERBOSITY(false)

#define SM_SERIALIZATION_CHECKSAME_VERBOSE(THIS, OTHER, VERBOSE) \
    SET_CHECKSAME_VERBOSITY(VERBOSE) && \
    SM_SERIALIZATION_CHECKSAME_IMPL(THIS, OTHER)

#define SM_SERIALIZATION_CHECKSAME_IMPL(THIS, OTHER) \
    (sm::serialization::internal::checkTypeIsNotOpencvMat<\
        typename sm::common::StripConstReference<decltype(OTHER)>::result_t, \
        false>::value &&  /*for opencvMats we have to use
        sm::opencv::isBinaryEqual otherwise this code has to depend on opencv*/ \
    sm::serialization::internal::isSame<\
    sm::serialization::internal::HasIsBinaryEqual<\
    /*first run the test of equality: either isBinaryEqual or op==*/ \
    typename sm::common::StripConstReference<decltype(OTHER)>::result_t>::value, \
    typename sm::common::StripConstReference<decltype(OTHER)>::result_t >::eval(THIS, OTHER)) ? \
        true : /*return true if good*/ \
    (IS_CHECKSAME_CURRENTLY_VERBOSE ? \
        (std::cout <<  "*** Validation failed on " << #OTHER << ":\n"<< \
        /* If not true, check whether VERBOSE and then try to output the
         * failed values using operator<<*/  \
    sm::serialization::internal::streamIf<\
    sm::serialization::internal::HasOStreamOperator<std::ostream, \
    /*here we check whether operator<< is available*/ \
    typename sm::common::StripConstReference<decltype(OTHER)>::result_t>::value, \
    typename sm::common::StripConstReference<decltype(OTHER)>::result_t >::eval(THIS) << \
    "other:\n" << sm::serialization::internal::streamIf<\
    sm::serialization::internal::HasOStreamOperator<std::ostream,\
    typename sm::common::StripConstReference<decltype(OTHER)>::result_t>::value, \
    typename sm::common::StripConstReference<decltype(OTHER)>::result_t>::eval(OTHER) \
    << "\nat " << __PRETTY_FUNCTION__ << /* Print the function where this happened.*/ \
    " In: " << __FILE__ << ":" << __LINE__ << std::endl << std::endl) && false : false)

#define SM_SERIALIZATION_CHECKMEMBERSSAME_VERBOSE(OTHER, MEMBER, VERBOSE) \
    (SET_CHECKSAME_VERBOSITY(VERBOSE) && \
        SM_SERIALIZATION_CHECKMEMBERSSAME_IMPL(OTHER, MEMBER)

#define SM_SERIALIZATION_CHECKMEMBERSSAME_IMPL(OTHER, MEMBER) \
    ((sm::serialization::internal::checkTypeIsNotOpencvMat<\
        typename sm::common::StripConstReference<decltype(OTHER)>::result_t,\
        false>::value) &&  /* For opencvMats we have to use sm::opencv::isBinaryEqual
        otherwise this code has to depend on opencv.*/\
    (sm::serialization::internal::isSame<\
        sm::serialization::internal::HasIsBinaryEqual<\
        typename sm::common::StripConstReference<decltype(MEMBER)>::result_t>::value, \
    typename sm::common::StripConstReference<\
    decltype(MEMBER)>::result_t >::eval(this->MEMBER, OTHER.MEMBER))) ? \
        true : (IS_CHECKSAME_CURRENTLY_VERBOSE ? \
            (std::cout <<  "*** Validation failed on " << #MEMBER << ":\n"<< \
        sm::serialization::internal::streamIf<\
        sm::serialization::internal::HasOStreamOperator<std::ostream,\
        typename sm::common::StripConstReference<decltype(MEMBER)>::result_t>::value, \
    typename sm::common::StripConstReference<decltype(MEMBER)>::result_t >::eval(this->MEMBER) << \
    "\nother:\n" << sm::serialization::internal::streamIf<\
    sm::serialization::internal::HasOStreamOperator<std::ostream,\
    typename sm::common::StripConstReference<decltype(MEMBER)>::result_t>::value, \
    typename sm::common::StripConstReference<decltype(MEMBER)>::result_t >::eval(OTHER.MEMBER) \
    << "\nat " << __PRETTY_FUNCTION__ << \
    " In: " << __FILE__ << ":" << __LINE__ << std::endl << std::endl) && false : false)

// This is some internal default macro parameter deduction.
#define SM_SERIALIZATION_GET_3RD_ARG(arg1, arg2, arg3, ...) arg3
#define SM_SERIALIZATION_GET_4TH_ARG(arg1, arg2, arg3, arg4, ...) arg4

#define SM_SERIALIZATION_MACRO_CHOOSER_MEMBER_SAME(...) \
    SM_SERIALIZATION_GET_4TH_ARG(__VA_ARGS__, SM_SERIALIZATION_CHECKMEMBERSSAME_VERBOSE,\
                                 SM_SERIALIZATION_CHECKMEMBERSSAME_IMPL)
#define SM_SERIALIZATION_MACRO_CHOOSER_SAME(...) \
    SM_SERIALIZATION_GET_4TH_ARG(__VA_ARGS__, SM_SERIALIZATION_CHECKSAME_VERBOSE,\
                                 SM_SERIALIZATION_CHECKSAME_IMPL)

//\brief This macro checks this->MEMBER against OTHER.MEMBER  with the
// appropriate IsBinaryEqual or operator==.
// Pointers and boost::shared_ptr are handled automatically.
#define SM_CHECKMEMBERSSAME(...) \
  SM_SERIALIZATION_MACRO_CHOOSER_MEMBER_SAME(__VA_ARGS__)(__VA_ARGS__)

//\brief This macro checks THIS against OTHER with the appropriate
// IsBinaryEqual or operator==.
// Pointers and boost::shared_ptr are handled automatically.
#define SM_CHECKSAME(...) \
  SM_SERIALIZATION_MACRO_CHOOSER_SAME(__VA_ARGS__)(__VA_ARGS__)

#endif //SM_SERIALIZATION_MACROS_HPP
