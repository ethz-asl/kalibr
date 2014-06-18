#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/logging.hpp>
#include <sm/logging/LoggingEvent.hpp>


void smLogNamed(const std::string & name, sm::logging::Level level, 
              const std::string & file, int line, const std::string & function, 
              const std::string & message)
{

    boost::shared_ptr<sm::logging::Logger> logger = sm::logging::getLogger();
    sm::logging::LoggingEvent event(name.c_str(),
                                    level,
                                    file.c_str(), 
                                    line, 
                                    function.c_str(),
                                    message.c_str(),
                                    logger->currentTimeString());
    
    logger->log(event);

}

void smLog(sm::logging::Level level, const std::string & file, int line, const std::string & function, 
         const std::string & message)
{
    smLogNamed(SMCONSOLE_DEFAULT_NAME,level,file,line,function,message);

}

void exportLogging()
{
    
    using namespace boost::python;
    using namespace sm::logging;
    
    enum_<Level>("LoggingLevel")
        .value("Debug",levels::Debug)
        .value("Info",levels::Info)
        .value("Warn",levels::Warn)
        .value("Error",levels::Error)
        .value("Fatal",levels::Fatal)
        ;

    // sm::logging::levels::Level getLevel();
    def("getLoggingLevel",&getLevel);
    // void setLevel( sm::logging::levels::Level level );        
    def("setLoggingLevel", &setLevel);
    // void setLogger( boost::shared_ptr<Logger> logger );
    def("setLogger", &setLogger);
    // boost::shared_ptr<Logger> getLogger();z
    def("getLogger", &getLogger);
    // bool isNamedStreamEnabled( const std::string & name );
    def("isNamedLoggingStreamEnabled", &isNamedStreamEnabled);
    // void enableNamedStream( const std::string & name );
    def("enableNamedLoggingStream", &enableNamedStream);
    // void disableNamedStream( const std::string & name );
    def("disableNamedLoggingStream", &disableNamedStream);
    def("rawLog",&smLog,"log(level,file,line,function,message)");
    def("rawLogNamed", &smLogNamed, "logNamed(level, file, line, function, message)");

    

    // Logging Event

    // Log

}
