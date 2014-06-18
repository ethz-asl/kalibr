#ifndef SM_LOGGING_HPP
#define SM_LOGGING_HPP

#include <sm/logging/console.hpp>


namespace sm {
    namespace logging {
        
        class Logger;

        sm::logging::levels::Level getLevel();
        void setLevel( sm::logging::levels::Level level );        
        void setLogger( boost::shared_ptr<Logger> logger );
        boost::shared_ptr<Logger> getLogger();
        bool isNamedStreamEnabled( const std::string & name );
        void enableNamedStream( const std::string & name );
        void disableNamedStream( const std::string & name );

        
        
    } // namespace logging
} // namespace sm

#define SM_INIT_LOGGING() \
  sm::logging::enableNamedStream( SMCONSOLE_NAME_PREFIX );



#endif /* SM_LOGGING_HPP */
