#ifndef SM_LOGGING_EVENT_HPP
#define SM_LOGGING_EVENT_HPP

#include <sm/logging/Levels.hpp>
#include <vector>
#include <string>


namespace sm {
    namespace logging {

        struct LoggingEvent
        {
            LoggingEvent(const char* streamName,
                         Level level,
                         const char* file, int line, 
                         const char* function,
                         const char * message,
                         const std::string & timestring) :
                streamName(streamName),
                level(level),
                file(file),
                line(line),
                function(function),
                message(message),
                timestring(timestring)
                {}

            const char * streamName;
            Level level;         
            const char * file;
            int line;            
            const char * function;
            const char * message;
            std::string timestring;
        };
        
        
    } // namespace logging
} // namespace sm


#endif /* SM_LOGGING_EVENT_HPP */
