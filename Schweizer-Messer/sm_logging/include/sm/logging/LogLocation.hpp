#ifndef SM_LOGGING_LOG_LOCATION_HPP
#define SM_LOGGING_LOG_LOCATION_HPP

namespace sm {
    namespace logging {
        struct LogLocation
        {
            LogLocation() : _initialized(false), _loggerEnabled(false),
                            _level(::sm::logging::levels::Count) {}

            bool _initialized;
            bool _loggerEnabled;
            ::sm::logging::Level _level;
            std::string _streamName;
        };

    } // namespace logging
} // namespace sm


#endif /* SM_LOGGING_LOG_LOCATION_HPP */
