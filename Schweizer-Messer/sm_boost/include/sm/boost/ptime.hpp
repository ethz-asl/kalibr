#ifndef SM_PTIME_HPP
#define SM_PTIME_HPP
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <sys/time.h>

namespace sm {
    
    timeval ptimeToTimeval(const boost::posix_time::ptime time)
    {
        static const boost::posix_time::ptime UNIX_EPOCH(boost::gregorian::date(1970,1,1));
        
        // \todo: check for special values.
        boost::posix_time::time_duration duration( time - UNIX_EPOCH );
        timeval tv;
        tv.tv_sec = duration.total_seconds();
        tv.tv_usec = duration.fractional_seconds();
        
        
        
    }

    

} // namespace sm


#endif /* SM_PTIME_HPP */
