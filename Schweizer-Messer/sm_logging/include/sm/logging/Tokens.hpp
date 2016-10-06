#ifndef SM_TOKENS_HPP
#define SM_TOKENS_HPP

#include <sm/logging/LoggingEvent.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <boost/thread.hpp>

#ifdef WIN32
	#define COLOR_NONE ""
	#define COLOR_NORMAL ""
	#define COLOR_RED ""
	#define COLOR_GREEN ""
	#define COLOR_YELLOW ""
#else
	#define COLOR_NONE ""
	#define COLOR_NORMAL "\033[0m"
	#define COLOR_RED "\033[31m"
  #define COLOR_LIGHT_RED "\033[91m"
	#define COLOR_GREEN "\033[32m"
  #define COLOR_LIGHT_GREEN "\033[92m"
	#define COLOR_YELLOW "\033[33m"
#endif

namespace sm {
    namespace logging {
        struct Token
        {
            virtual ~Token() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event) = 0;
        };
        typedef boost::shared_ptr<Token> TokenPtr;
        typedef std::vector<TokenPtr> V_Token;

        
        struct FixedToken : public Token
        {
            FixedToken(const std::string& str)
                : str_(str)
                {}
            virtual ~FixedToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent&)
                {
                    return str_.c_str();
                }

            std::string str_;
        };

        struct FixedMapToken : public Token
        {
            FixedMapToken(const std::string& str, std::map<std::string, std::string> & smap)
                : str_(str), smap_(smap)
                {}
            virtual ~FixedMapToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent&)
                {
                    std::map<std::string, std::string>::iterator it = smap_.find(str_);
                    if (it == smap_.end())
                    {
                        return ("${" + str_ + "}").c_str();
                    }

                    return it->second.c_str();
                }

            std::string str_;
            std::map<std::string, std::string> & smap_;
            
        };

        struct PlaceHolderToken : public Token
        {
            virtual ~PlaceHolderToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent&)
                {
                    return "PLACEHOLDER";
                }
        };

        struct SeverityToken : public Token
        {
            virtual ~SeverityToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event)
                {
                    if (event.level == SMCONSOLE_SEVERITY_FATAL)
                    {
                        return "FATAL";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_ERROR)
                    {
                        return "ERROR";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_WARN)
                    {
                        return " WARN";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_INFO)
                    {
                        return " INFO";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_DEBUG)
                    {
                        return "DEBUG";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_FINE)
                    {
                        return " FINE";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_TRACE)
                    {
                        return "TRACE";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_FINER)
                    {
                        return "FINER";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_VERBOSE)
                    {
                        return "VERBO";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_FINEST)
                    {
                        return "FINES";
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_ALL)
                    {
                        return "  ALL";
                    }

                    return "UNKNO";
                }
        };

        struct MessageToken : public Token
        {
            virtual ~MessageToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event)
                {
                    // \todo Does this still work on Windows?
// #ifdef _MSC_VER
//                     // has to handle LogString with wchar types.
//                     LOG4CXX_ENCODE_CHAR(ret, event.getMessage());
//                     return ret;
// #else
                    return event.message;
//#endif
                }
        };

        struct StreamNameToken : public Token
        {
            virtual ~StreamNameToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event)
                {
                    return event.streamName;
                }
        };

    
        struct TimeToken : public Token
        {
            virtual ~TimeToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent&e)
                {
                    return e.timestring;
                }
        };

        struct ThreadToken : public Token
        {
            virtual ~ThreadToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent&)
                {
                    std::stringstream ss;
                    ss << boost::this_thread::get_id();
                    return ss.str();
                }
        };

        struct FileToken : public Token
        {
            virtual ~FileToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event)
                {
                    return event.file;
                }
        };

        struct FunctionToken : public Token
        {
            virtual ~FunctionToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event)
                {
                    return event.function;
                }
        };

        struct LineToken : public Token
        {
            virtual ~LineToken() {

            }
            virtual std::string getString(const ::sm::logging::LoggingEvent& event)
                {
                    std::stringstream ss;
                    ss << event.line;
                    return ss.str();
                }
        };



    } // namespace logging
} // namespace sm


#endif /* SM_TOKENS_HPP */
