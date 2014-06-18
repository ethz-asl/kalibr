#include <sm/logging/LoggingGlobals.hpp>
#include <sm/logging/StdOutLogger.hpp>

namespace sm {
    namespace logging {

        LoggingGlobals g_logging_globals;



        LoggingGlobals::LoggingGlobals()
        {
            _shutting_down = false;
            _logger.reset( new StdOutLogger() );
            _level = levels::Info;
            enableNamedStream(SMCONSOLE_DEFAULT_NAME);
            _globalCharBuffer.resize(4096);
        }

        LoggingGlobals::~LoggingGlobals()
        {

        }
                        
        void LoggingGlobals::shutdown()
        {
            _shutting_down = true;
        }
            
        /**
         * \brief Registers a logging location with the system.
         *
         * This is used for the case where a logger's verbosity level changes, and we need to reset the enabled status of
         * all the logging statements.
         * @param loc The location to add
         */
        void LoggingGlobals::registerLogLocation(LogLocation* loc) {
            boost::mutex::scoped_lock lock(_locations_mutex);
            _log_locations.push_back(loc);
        }

        void LoggingGlobals::checkLogLocationEnabledNoLock(LogLocation* loc) {
            loc->_loggerEnabled = loc->_level >= _level && (loc->_streamName.size() == 0 || isNamedStreamEnabled(loc->_streamName));
        }

        void LoggingGlobals::initializeLogLocation(LogLocation* loc, const std::string& name, Level level){
            boost::mutex::scoped_lock lock(_locations_mutex);
                
            if (loc->_initialized)
            {
                return;
            }

            loc->_streamName = name;
            loc->_level = level;
                
            _log_locations.push_back(loc);
                
            checkLogLocationEnabledNoLock(loc);
            
            loc->_initialized = true;
        }

        void LoggingGlobals::setLogLocationLevel(LogLocation* loc, Level level)
        {
            boost::mutex::scoped_lock lock(_locations_mutex);
            loc->_level = level;
        }

        void LoggingGlobals::checkLogLocationEnabled(LogLocation* loc)
        {
            boost::mutex::scoped_lock lock(_locations_mutex);
            checkLogLocationEnabledNoLock(loc);
        }



        /**
         * \brief Tells the system that a logger's level has changed
         *
         * This must be called if a log4cxx::Logger's level has been changed in the middle of an application run.
         * Because of the way the static guard for enablement works, if a logger's level is changed and this
         * function is not called, only logging statements which are first hit *after* the change will be correct wrt
         * that logger.
         */
        void LoggingGlobals::notifyLoggerLevelsChanged()
        {
            boost::mutex::scoped_lock lock(_locations_mutex);
                    
            V_LogLocation::iterator it = _log_locations.begin();
            V_LogLocation::iterator end = _log_locations.end();
            for ( ; it != end; ++it )
            {
                LogLocation* loc = *it;
                checkLogLocationEnabledNoLock(loc);
            }
        }

        
        bool LoggingGlobals::isNamedStreamEnabled( const std::string & name ){
            boost::recursive_mutex::scoped_lock lock(_named_stream_mutex);
            std::set< std::string >::iterator it = _namedStreamsEnabled.find(name);
            return it != _namedStreamsEnabled.end();
                
        }

        void LoggingGlobals::enableNamedStream( const std::string & name ) {
            boost::recursive_mutex::scoped_lock lock(_named_stream_mutex);
            _namedStreamsEnabled.insert(name);
            notifyLoggerLevelsChanged();
        }

        void LoggingGlobals::disableNamedStream( const std::string & name ) {
            boost::recursive_mutex::scoped_lock lock(_named_stream_mutex);
            std::set< std::string >::iterator it = _namedStreamsEnabled.find(name);
            if(it != _namedStreamsEnabled.end())
            {
                _namedStreamsEnabled.erase(it);
                notifyLoggerLevelsChanged();
            }
        }




        void LoggingGlobals::vformatToBuffer(const char* fmt, va_list args)
        {
#ifdef _MSC_VER
            va_list arg_copy = args; // dangerous?
#else
            va_list arg_copy;
            va_copy(arg_copy, args);
#endif
            
#ifdef _MSC_VER
            size_t total = vsnprintf_s(&_globalCharBuffer[0], _globalCharBuffer.size(), _globalCharBuffer.size(), fmt, args);
#else
            size_t total = vsnprintf(&_globalCharBuffer[0], _globalCharBuffer.size(), fmt, args);
#endif
            if (total >= _globalCharBuffer.size())
            {
                _globalCharBuffer.resize(total + 2);
#ifdef _MSC_VER
                total = vsnprintf_s(&_globalCharBuffer[0], _globalCharBuffer.size(), _globalCharBuffer.size(), fmt, arg_copy);
#else
                total = vsnprintf(&_globalCharBuffer[0], _globalCharBuffer.size(), fmt, arg_copy);
#endif
               
            }
            va_end(arg_copy);

        }

        void LoggingGlobals::formatToBuffer(const char* fmt, ...)
        {
            va_list args;
            va_start(args, fmt);

            vformatToBuffer(fmt, args);

            va_end(args);
        }


        void LoggingGlobals::print(const char * streamName, Level level, const char* file, int line, const char* function, const char* fmt, ...)
        {
            if (_shutting_down)
                return;

            if (_printing_thread_id == boost::this_thread::get_id())
            {
                fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
                return;
            }

            boost::mutex::scoped_lock lock(_print_mutex);

            _printing_thread_id = boost::this_thread::get_id();

            va_list args;
            va_start(args, fmt);

            vformatToBuffer(fmt, args);

            va_end(args);

            try
            {
                _logger->log( LoggingEvent( streamName, level, file, line, function, &_globalCharBuffer[0], _logger->currentTimeString() ) );
            }
            catch (std::exception& e)
            {
                fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
            }
                    

            _printing_thread_id = boost::thread::id();
        }

        void LoggingGlobals::print(const char * streamName,  Level level, 
                                   vectorstream & ss, const char* file, int line, const char* function)
        {
            if (_shutting_down)
                return;

            if (_printing_thread_id == boost::this_thread::get_id())
            {
                fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
                return;
            }

            std::vector<char> str;
            ss.swap_vector(str);
            // make sure the string is null terminated.
            str.push_back('\0');
            
            boost::mutex::scoped_lock lock(_print_mutex);

            _printing_thread_id = boost::this_thread::get_id();
            try
            {
                _logger->log( LoggingEvent( streamName, level, file, line, function, &str[0], _logger->currentTimeString() ) );
            }
            catch (std::exception& e)
            {
                fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
            }
                    
            _printing_thread_id = boost::thread::id();
        }


        sm::logging::levels::Level LoggingGlobals::getLevel()
        {
            // \todo is this thread safe?
            return _level;
        }

        void LoggingGlobals::setLevel( sm::logging::levels::Level level )
        {
            // \todo is this thread safe?
            if(level != _level)
            {
                _level = level;
                notifyLoggerLevelsChanged();
            }
        }

        void LoggingGlobals::setLogger( boost::shared_ptr<Logger> logger )
        {
            if(logger.get())
            {
                _logger = logger;
            }
        }
        boost::shared_ptr<Logger> LoggingGlobals::getLogger()
        {
            return _logger;
        }

        sm::logging::levels::Level getLevel()
        {
            return g_logging_globals.getLevel();
        }
        void setLevel( sm::logging::levels::Level level )
        {
            g_logging_globals.setLevel(level);
        }
        void setLogger( boost::shared_ptr<Logger> logger )
        {
            g_logging_globals.setLogger(logger);
        }
        boost::shared_ptr<Logger> getLogger()
        {
            return g_logging_globals.getLogger();
        }
        bool isNamedStreamEnabled( const std::string & name )
        {
            return g_logging_globals.isNamedStreamEnabled( name );
        }
        void enableNamedStream( const std::string & name )
        {
            g_logging_globals.enableNamedStream( name );
        }
        void disableNamedStream( const std::string & name )
        {
            g_logging_globals.disableNamedStream( name );
        }

        
    } // namespace logging
} // namespace sm
