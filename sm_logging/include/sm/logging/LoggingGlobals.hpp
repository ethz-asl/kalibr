#ifndef SM_LOGGING_GLOBALS_HPP
#define SM_LOGGING_GLOBALS_HPP

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <stdarg.h>
#include <vector>
#include <set>

#include <sm/logging/macros.h>
#include <sm/logging/Levels.hpp>
#include <sm/logging/LogLocation.hpp>
#include <sm/logging/LoggingEvent.hpp>
#include <boost/interprocess/streams/vectorstream.hpp>
#include <boost/thread.hpp>
#include <sm/logging/Logger.hpp>



namespace sm {
    namespace logging {

        typedef boost::interprocess::basic_vectorstream<std::vector<char> >    vectorstream;
        
        class Logger;

        class LoggingGlobals
        {
        public:
            typedef std::vector<LogLocation*> V_LogLocation;


            LoggingGlobals();
            virtual ~LoggingGlobals();
                        
            void shutdown();
            
            /**
             * \brief Registers a logging location with the system.
             *
             * This is used for the case where a logger's verbosity level changes, and we need to reset the enabled status of
             * all the logging statements.
             * @param loc The location to add
             */
            void registerLogLocation(LogLocation* loc);

            void checkLogLocationEnabledNoLock(LogLocation* loc);

            void initializeLogLocation(LogLocation* loc, const std::string& name, Level level);

            void setLogLocationLevel(LogLocation* loc, Level level);

            void checkLogLocationEnabled(LogLocation* loc);



            /**
             * \brief Tells the system that a logger's level has changed
             *
             * This must be called if a log4cxx::Logger's level has been changed in the middle of an application run.
             * Because of the way the static guard for enablement works, if a logger's level is changed and this
             * function is not called, only logging statements which are first hit *after* the change will be correct wrt
             * that logger.
             */
            void notifyLoggerLevelsChanged();

        
            bool isNamedStreamEnabled( const std::string & name );

            void enableNamedStream( const std::string & name );

            void disableNamedStream( const std::string & name );

            sm::logging::levels::Level getLevel();
            void setLevel( sm::logging::levels::Level level );        
            void setLogger( boost::shared_ptr<Logger> logger );
            boost::shared_ptr<Logger> getLogger();


            void vformatToBuffer(const char* fmt, va_list args);
            void formatToBuffer(const char* fmt, ...);


            
            void print(const char * streamName, Level level, const char* file, int line, const char* function, const char* fmt, ... ) SMCONSOLE_PRINTF_ATTRIBUTE(7, 8);
            void print(const char * streamName,  Level level, 
                       vectorstream & ss, const char* file, int line, const char* function);

            Level _level;
            
            boost::shared_ptr<Logger> _logger;

            bool _shutting_down;

            boost::mutex _init_mutex;


            boost::mutex _locations_mutex;
            V_LogLocation _log_locations;

            boost::recursive_mutex _named_stream_mutex;
            std::set< std::string > _namedStreamsEnabled;
            
            std::vector<char> _globalCharBuffer;
            boost::mutex _print_mutex;
            boost::thread::id _printing_thread_id;


        };

        extern LoggingGlobals g_logging_globals;

    } // namespace logging
} // namespace sm


#endif /* SM_LOGGING_GLOBALS_HPP */
