#ifndef SM_FORMATTER_HPP
#define SM_FORMATTER_HPP

#include "Tokens.hpp"
#include "LoggingEvent.hpp"
#include <boost/regex.hpp>

namespace sm {
    namespace logging {
        

        struct Formatter
        {

            Formatter();
            virtual ~Formatter();


            TokenPtr createTokenFromType(const std::string& type);
            void init(const char* fmt);
            void print(const ::sm::logging::LoggingEvent& event, std::ostream & ss);
            
            std::string format_;
            V_Token tokens_;
            bool doColor_;
            typedef std::map<std::string, std::string> M_string;
            M_string extra_fixed_tokens_;


        };
       

    } // namespace logging
} // namespace sm


#endif /* SM_FORMATTER_HPP */
