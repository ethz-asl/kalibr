/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#ifndef SMLIB_MACROS_H_INCLUDED
#define SMLIB_MACROS_H_INCLUDED

#if defined(__GNUC__)
#define SM_DEPRECATED __attribute__((deprecated))
#define SM_FORCE_INLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
#define SM_DEPRECATED
#define SM_FORCE_INLINE __forceinline
#else
#define SM_DEPRECATED
#define SM_FORCE_INLINE inline
#endif

/*
  Windows import/export and gnu http://gcc.gnu.org/wiki/Visibility
  macros.
 */
#if defined(_MSC_VER)
    #define SM_HELPER_IMPORT __declspec(dllimport)
    #define SM_HELPER_EXPORT __declspec(dllexport)
    #define SM_HELPER_LOCAL
#elif __GNUC__ >= 4
    #define SM_HELPER_IMPORT __attribute__ ((visibility("default")))
    #define SM_HELPER_EXPORT __attribute__ ((visibility("default")))
    #define SM_HELPER_LOCAL  __attribute__ ((visibility("hidden")))
#else
    #define SM_HELPER_IMPORT
    #define SM_HELPER_EXPORT
    #define SM_HELPER_LOCAL
#endif

// Ignore warnings about import/exports when deriving from std classes.
#ifdef _MSC_VER
  #pragma warning(disable: 4251)
  #pragma warning(disable: 4275)
#endif


#ifdef __GNUC__
#if __GNUC__ >= 3
#define SMCONSOLE_PRINTF_ATTRIBUTE(a, b) __attribute__ ((__format__ (__printf__, a, b)));
#endif
#endif

#ifndef SMCONSOLE_PRINTF_ATTRIBUTE
#define SMCONSOLE_PRINTF_ATTRIBUTE(a, b)
#endif

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef SM_BUILD_SHARED_LIBS // sm is being built around shared libraries
  #ifdef smconsole_EXPORTS // we are building a shared lib/dll
    #define SMCONSOLE_DECL SM_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define SMCONSOLE_DECL SM_HELPER_IMPORT
  #endif
#else // sm is being built around static libraries
  #define SMCONSOLE_DECL
#endif

#ifdef WIN32
#define SM_LIKELY(x)       (x)
#define SM_UNLIKELY(x)     (x)
#else
#define SM_LIKELY(x)       __builtin_expect((x),1)
#define SM_UNLIKELY(x)     __builtin_expect((x),0)
#endif

#if defined(MSVC)
#define __SMCONSOLE_FUNCTION__ __FUNCSIG__
#elif defined(__GNUC__)
#define __SMCONSOLE_FUNCTION__ __PRETTY_FUNCTION__
#else
#define __SMCONSOLE_FUNCTION__ ""
#endif


#ifdef SM_PACKAGE_NAME
#define SMCONSOLE_PACKAGE_NAME SM_PACKAGE_NAME
#else
#define SMCONSOLE_PACKAGE_NAME "sm"
#endif

#define SMCONSOLE_NAME_PREFIX SMCONSOLE_PACKAGE_NAME
#define SMCONSOLE_DEFAULT_NAME SMCONSOLE_NAME_PREFIX


/**
 * \def SMCONSOLE_MIN_SEVERITY
 *
 * Define SMCONSOLE_MIN_SEVERITY=SMCONSOLE_SEVERITY_[ALL|FINEST|VERBOSE|FINER|TRACE|FINE|DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity
 */
#ifndef SMCONSOLE_MIN_SEVERITY
#define SMCONSOLE_MIN_SEVERITY SMCONSOLE_SEVERITY_ALL
#endif



#endif

