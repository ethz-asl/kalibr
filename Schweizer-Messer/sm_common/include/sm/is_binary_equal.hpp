#ifndef SM_IS_BINARY_EQUAL_HPP
#define SM_IS_BINARY_EQUAL_HPP

#  warning "This header is deprecated. Please use: the macros in sm/serialization_macros.hpp"

#include <sm/serialization_macros.hpp>

namespace sm {
    
    template<typename T>
    inline bool isBinaryEqual(const T * t1, const T * t2)
    {
        return SM_CHECKSAME(t1, t2);
    }

    template<typename T>
    inline bool isBinaryEqual(const boost::shared_ptr<T> & t1, const boost::shared_ptr<T> & t2)
    {
        return SM_CHECKSAME(t1, t2);
    }


} // namespace sm


#endif /* SM_IS_BINARY_EQUAL_HPP */
