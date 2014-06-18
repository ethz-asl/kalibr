#ifndef ASLAM_BACKEND_EXCEPTIONS_HPP
#define ASLAM_BACKEND_EXCEPTIONS_HPP

#include <sm/assert_macros.hpp>

namespace aslam {
    
    SM_DEFINE_EXCEPTION(Exception,std::runtime_error);
    SM_DEFINE_EXCEPTION(IndexOutOfBoundsException,Exception);
    SM_DEFINE_EXCEPTION(NotImplementedException,Exception);
    SM_DEFINE_EXCEPTION(InvalidArgumentException,Exception);
    SM_DEFINE_EXCEPTION(UnsupportedOperationException,Exception);

} // namespace aslam


#endif /* ASLAM_BACKEND_EXCEPTIONS_HPP */
