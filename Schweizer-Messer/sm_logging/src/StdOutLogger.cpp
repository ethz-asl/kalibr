#include <sm/logging/StdOutLogger.hpp>
#include <boost/cstdint.hpp>
#include <sstream>
#ifdef WIN32
#include <windows.h>
#endif
#include <sys/time.h>
#include <iostream>
#include <sm/logging/LoggingEvent.hpp>

namespace sm {
    namespace logging {

        StdOutLogger::StdOutLogger(){}
        StdOutLogger::~StdOutLogger(){}
            
        
        void StdOutLogger::logImplementation(const LoggingEvent & event)
        {
            formatter.print(event,std::cout);
            
        }


    } // namespace logging
} // namespace sm
