#ifndef SM_LOGGING_LEVELS_HPP
#define SM_LOGGING_LEVELS_HPP

// These allow you to compile-out everything below a certain severity level if necessary
#define SMCONSOLE_SEVERITY_ALL 0
#define SMCONSOLE_SEVERITY_FINEST 1
#define SMCONSOLE_SEVERITY_VERBOSE 2
#define SMCONSOLE_SEVERITY_FINER 3
#define SMCONSOLE_SEVERITY_TRACE 4
#define SMCONSOLE_SEVERITY_FINE 5
#define SMCONSOLE_SEVERITY_DEBUG 6
#define SMCONSOLE_SEVERITY_INFO 7
#define SMCONSOLE_SEVERITY_WARN 8
#define SMCONSOLE_SEVERITY_ERROR 9
#define SMCONSOLE_SEVERITY_FATAL 10
#define SMCONSOLE_SEVERITY_NONE 11


namespace sm {
    namespace logging {
        
        namespace levels
        {
            enum Level
            {
                All,
                Finest,
                Verbose,
                Finer,
                Trace,
                Fine,
                Debug,
                Info,
                Warn,
                Error,
                Fatal,

                Count
            };
        }

        typedef levels::Level Level;

    } // namespace logging
} // namespace sm


#endif /* SM_LOGGING_LEVELS_HPP */
