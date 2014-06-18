#include <sm/logging/Formatter.hpp>

namespace sm {
    namespace logging {
        
        Formatter::~Formatter(){}

            TokenPtr Formatter::createTokenFromType(const std::string& type)
                {
                    if (type == "severity")
                    {
                        return TokenPtr(new SeverityToken());
                    }
                    else if (type == "message")
                    {
                        return TokenPtr(new MessageToken());
                    }
                    else if (type == "time")
                    {
                        return TokenPtr(new TimeToken());
                    }
                    else if (type == "thread")
                    {
                        return TokenPtr(new ThreadToken());
                    }
                    else if (type == "file")
                    {
                        return TokenPtr(new FileToken());
                    }
                    else if (type == "line")
                    {
                        return TokenPtr(new LineToken());
                    }
                    else if (type == "function")
                    {
                        return TokenPtr(new FunctionToken());
                    }
                    else if (type == "streamname")
                    {
                        return TokenPtr(new StreamNameToken());
                    }

                    return TokenPtr(new FixedMapToken(type, extra_fixed_tokens_));
                }


            void Formatter::init(const char* fmt)
                {
                    format_ = fmt;

                    boost::regex e("\\$\\{([a-z|A-Z]+)\\}");
                    boost::match_results<std::string::const_iterator> results;
                    std::string::const_iterator start, end;
                    start = format_.begin();
                    end = format_.end();
                    bool matched_once = false;
                    std::string last_suffix;
                    while (boost::regex_search(start, end, results, e))
                    {
#if 0
                        for (size_t i = 0; i < results.size(); ++i)
                        {
                            std::cout << i << "|" << results.prefix() << "|" <<  results[i] << "|" << results.suffix() << std::endl;
                        }
#endif

                        std::string token = results[1];
                        last_suffix = results.suffix();
                        tokens_.push_back(TokenPtr(new FixedToken(results.prefix())));
                        tokens_.push_back(createTokenFromType(token));

                        start = results[0].second;
                        matched_once = true;
                    }

                    if (matched_once)
                    {
                        tokens_.push_back(TokenPtr(new FixedToken(last_suffix)));
                    }
                    else
                    {
                        tokens_.push_back(TokenPtr(new FixedToken(format_)));
                    }
                }

            void Formatter::print(const ::sm::logging::LoggingEvent& event, std::ostream & ss)
                {
                    const char* color = COLOR_NORMAL;
                    //FILE * f = stdout;

                    if (event.level == SMCONSOLE_SEVERITY_FATAL)
                    {
                        color = COLOR_LIGHT_RED;
                        //f = stderr;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_ERROR)
                    {
                        color = COLOR_RED;
                        //f = stderr;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_WARN)
                    {
                        color = COLOR_YELLOW;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_INFO)
                    {
                        color = COLOR_NORMAL;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_DEBUG)
                    {
                        color = COLOR_GREEN;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_FINE)
                    {
                        color = COLOR_LIGHT_GREEN;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_TRACE)
                    {
                        color = COLOR_LIGHT_GREEN;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_FINER)
                    {
                        color = COLOR_LIGHT_GREEN;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_VERBOSE)
                    {
                        color = COLOR_LIGHT_GREEN;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_FINEST)
                    {
                        color = COLOR_LIGHT_GREEN;
                    }
                    else if (event.level == SMCONSOLE_SEVERITY_ALL)
                    {
                        color = COLOR_LIGHT_GREEN;
                    }

                    if(doColor_)
                    {
                        ss << color;
                    }
                    V_Token::iterator it = tokens_.begin();
                    V_Token::iterator end = tokens_.end();
                    for (; it != end; ++it)
                    {
                        ss << (*it)->getString(event);
                    }
                    if(doColor_)
                    {
                        ss << COLOR_NORMAL;
                    }
                    ss << std::endl;

                }

        Formatter::Formatter() : doColor_(true){
                // Check for the format string environment variable
            char* format_string = NULL;
#ifdef _MSC_VER
                    _dupenv_s(&format_string, NULL, "SMCONSOLE_FORMAT");
#else
                format_string =  getenv("SMCONSOLE_FORMAT");
#endif
                if (!format_string)
                {
                    init("[${severity}] [${time}]: ${message}");
                }
                else
                {
                    init(format_string);
                }
                
            }


    } // namespace logging
} // namespace sm
