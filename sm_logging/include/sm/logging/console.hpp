/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Josh Faust

#ifndef SMCONSOLE_SMCONSOLE_H
#define SMCONSOLE_SMCONSOLE_H

#include <cstdio>
#include <sstream>

#include <cstdarg>
#include <sm/logging/macros.h>
#include <sm/logging/Levels.hpp>
#include <sm/logging/LoggingGlobals.hpp>
#include <boost/interprocess/streams/vectorstream.hpp>

namespace boost
{
    template<typename T> class shared_array;
}

namespace sm
{
    namespace logging {
        class Logger;
    } // namespace logging

    namespace logging
    {



    } // namespace logging
} // namespace sm





#define SMCONSOLE_DEFINE_LOCATION(cond, level, name)                    \
    static ::sm::logging::LogLocation loc;                              \
    if (SM_UNLIKELY(!loc._initialized))                                 \
    {                                                                   \
        ::sm::logging::g_logging_globals.initializeLogLocation(&loc, name, level); \
    }                                                                   \
    if (SM_UNLIKELY(loc._level != level))                               \
    {                                                                   \
        ::sm::logging::g_logging_globals.setLogLocationLevel(&loc, level);             \
        ::sm::logging::g_logging_globals.checkLogLocationEnabled(&loc);                \
    }                                                                   \
    bool enabled = loc._loggerEnabled && (cond);

#define SMCONSOLE_PRINT_AT_LOCATION(...)                      \
    ::sm::logging::g_logging_globals.print(loc._streamName.c_str(), loc._level, __FILE__, __LINE__, __SMCONSOLE_FUNCTION__, __VA_ARGS__)


#define SMCONSOLE_PRINT_STREAM_AT_LOCATION(args)                        \
    do                                                                  \
    {                                                                   \
        ::sm::logging::vectorstream ss;                                 \
        ss << args;                                                     \
        ::sm::logging::g_logging_globals.print(loc._streamName.c_str(), loc._level, ss, __FILE__, __LINE__, __SMCONSOLE_FUNCTION__); \
    } while (0)


/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with printf-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG_COND(cond, level, name, ...)             \
    do                                                  \
    {                                                   \
        SMCONSOLE_DEFINE_LOCATION(cond, level, name);   \
                                                        \
        if (SM_UNLIKELY(enabled))                       \
        {                                               \
            SMCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__);   \
        }                                               \
    } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with stream-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG_STREAM_COND(cond, level, name, args)     \
    do                                                  \
    {                                                   \
        SMCONSOLE_DEFINE_LOCATION(cond, level, name);   \
        if (SM_UNLIKELY(enabled))                       \
        {                                               \
            SMCONSOLE_PRINT_STREAM_AT_LOCATION(args);   \
        }                                               \
    } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG_ONCE(level, name, ...)                   \
    do                                                  \
    {                                                   \
        SMCONSOLE_DEFINE_LOCATION(true, level, name);   \
        static bool hit = false;                        \
        if (SM_UNLIKELY(enabled) && SM_UNLIKELY(!hit))  \
        {                                               \
            hit = true;                                 \
            SMCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__);   \
        }                                               \
    } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG_STREAM_ONCE(level, name, args)           \
    do                                                  \
    {                                                   \
        SMCONSOLE_DEFINE_LOCATION(true, level, name);   \
        static bool hit = false;                        \
        if (SM_UNLIKELY(enabled) && SM_UNLIKELY(!hit))  \
        {                                               \
            hit = true;                                 \
            SMCONSOLE_PRINT_STREAM_AT_LOCATION(args);   \
        }                                               \
    } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define SM_LOG_THROTTLE(rate, level, name, ...)                         \
    do                                                                  \
    {                                                                   \
        SMCONSOLE_DEFINE_LOCATION(true, level, name);                   \
        static double last_hit = 0.0;                                   \
        double now = ::sm::logging::g_logging_globals._logger->currentTimeSecondsUtc(); \
        if (SM_UNLIKELY(enabled) && SM_UNLIKELY(last_hit + rate <= now)) \
        {                                                               \
            last_hit = now;                                             \
            SMCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__);                   \
        }                                                               \
    } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define SM_LOG_STREAM_THROTTLE(rate, level, name, args)                 \
    do                                                                  \
    {                                                                   \
        SMCONSOLE_DEFINE_LOCATION(true, level, name);                   \
        static double last_hit = 0.0;                                   \
        double now = ::sm::logging::g_logging_globals._logger->currentTimeSecondsUtc(); \
        if (SM_UNLIKELY(enabled) && SM_UNLIKELY(last_hit + rate <= now)) \
        {                                                               \
            last_hit = now;                                             \
            SMCONSOLE_PRINT_STREAM_AT_LOCATION(args);                   \
        }                                                               \
    } while(0)


/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with stream-style formatting
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG_STREAM_NAME(name, level, args)                     \
    do                                                                  \
    {                                                                   \
        SMCONSOLE_DEFINE_LOCATION(true, level, name);                   \
        if (SM_UNLIKELY(enabled) )                                      \
        {                                                               \
            SMCONSOLE_PRINT_STREAM_AT_LOCATION(args);   \
        }                                                               \
    } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with printf-style formatting
 *
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG(level, name, ...) SM_LOG_COND(true, level, name, __VA_ARGS__)
/**
 * \brief Log to a given named logger at a given verbosity level, with stream-style formatting
 *
 * \param level One of the levels specified in ::sm::logging::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "sm.<package_name>".  Use SMCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define SM_LOG_STREAM(level, name, args) SM_LOG_STREAM_COND(true, level, name, args)

#include "macros_generated.hpp"

#endif // SMCONSOLE_SMCONSOLE_H
