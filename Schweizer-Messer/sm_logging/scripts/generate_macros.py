#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import roslib
roslib.load_manifest('sm_logging')

base_path = roslib.packages.get_pkg_dir('sm_logging')

def add_macro(f, caps_name, enum_name):
    f.write('#if (SMCONSOLE_MIN_SEVERITY > SMCONSOLE_SEVERITY_%s)\n' %(caps_name))
    f.write('#define SM_%s(...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM(args)\n' %(caps_name))
    f.write('#define SM_%s_NAMED(name, ...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_NAMED(name, args)\n' %(caps_name))
    f.write('#define SM_%s_COND(cond, ...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_COND(cond, args)\n' %(caps_name))
    f.write('#define SM_%s_COND_NAMED(cond, name, ...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_COND_NAMED(cond, name, args)\n' %(caps_name))
    
    f.write('#define SM_%s_ONCE(...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_ONCE(args)\n' %(caps_name))
    f.write('#define SM_%s_ONCE_NAMED(name, ...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_ONCE_NAMED(name, args)\n' %(caps_name))
    f.write('#define SM_%s_THROTTLE(rate, ...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_THROTTLE(rate, args)\n' %(caps_name))
    f.write('#define SM_%s_THROTTLE_NAMED(rate, name, ...)\n' %(caps_name))
    f.write('#define SM_%s_STREAM_THROTTLE_NAMED(rate, name, args)\n' %(caps_name))
    
    # f.write('#define SM_%s_FILTER(filter, ...)\n' %(caps_name))
    # f.write('#define SM_%s_STREAM_FILTER(filter, args)\n' %(caps_name))
    # f.write('#define SM_%s_FILTER_NAMED(filter, name, ...)\n' %(caps_name))
    # f.write('#define SM_%s_STREAM_FILTER_NAMED(filter, name, args)\n' %(caps_name))
    f.write('#else\n')
    f.write('#define SM_%s(...) SM_LOG(::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM(args) SM_LOG_STREAM(::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, args)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_NAMED(name, ...) SM_LOG(::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_NAMED(name, args) SM_LOG_STREAM(::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, args)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_COND(cond, ...) SM_LOG_COND(cond, ::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_COND(cond, args) SM_LOG_STREAM_COND(cond, ::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, args)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_COND_NAMED(cond, name, ...) SM_LOG_COND(cond, ::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_COND_NAMED(cond, name, args) SM_LOG_STREAM_COND(cond, ::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, args)\n' %(caps_name, enum_name))
    
    f.write('#define SM_%s_ONCE(...) SM_LOG_ONCE(::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_ONCE(args) SM_LOG_STREAM_ONCE(::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, args)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_ONCE_NAMED(name, ...) SM_LOG_ONCE(::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_ONCE_NAMED(name, args) SM_LOG_STREAM_ONCE(::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, args)\n' %(caps_name, enum_name))
    
    f.write('#define SM_%s_THROTTLE(rate, ...) SM_LOG_THROTTLE(rate, ::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_THROTTLE(rate, args) SM_LOG_STREAM_THROTTLE(rate, ::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, args)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_THROTTLE_NAMED(rate, name, ...) SM_LOG_THROTTLE(rate, ::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)\n' %(caps_name, enum_name))
    f.write('#define SM_%s_STREAM_THROTTLE_NAMED(rate, name, args) SM_LOG_STREAM_THROTTLE(rate, ::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, args)\n' %(caps_name, enum_name))
    
    # f.write('#define SM_%s_FILTER(filter, ...) SM_LOG_FILTER(filter, ::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, __VA_ARGS__)\n' %(caps_name, enum_name))
    # f.write('#define SM_%s_STREAM_FILTER(filter, args) SM_LOG_STREAM_FILTER(filter, ::sm::logging::levels::%s, SMCONSOLE_DEFAULT_NAME, args)\n' %(caps_name, enum_name))
    # f.write('#define SM_%s_FILTER_NAMED(filter, name, ...) SM_LOG_FILTER(filter, ::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)\n' %(caps_name, enum_name))
    # f.write('#define SM_%s_STREAM_FILTER_NAMED(filter, name, args) SM_LOG_STREAM_FILTER(filter, ::sm::logging::levels::%s, std::string(SMCONSOLE_NAME_PREFIX) + "." + name, args)\n' %(caps_name, enum_name))
    f.write('#endif\n\n')

f = open('%s/include/sm/logging/macros_generated.hpp' %(base_path), 'w')

f.write("// !!!!!!!!!!!!!!!!!!!!!!! This is a generated file, do not edit manually\n\n")

f.write('/*\n')
f.write(' * Copyright (c) 2008, Willow Garage, Inc.\n')
f.write(' * All rights reserved.\n')
f.write(' *\n')
f.write(' * Redistribution and use in source and binary forms, with or without\n')
f.write(' * modification, are permitted provided that the following conditions are met:\n')
f.write(' *\n')
f.write(' *     * Redistributions of source code must retain the above copyright\n')
f.write(' *       notice, this list of conditions and the following disclaimer.\n')
f.write(' *     * Redistributions in binary form must reproduce the above copyright\n')
f.write(' *       notice, this list of conditions and the following disclaimer in the\n')
f.write(' *       documentation and/or other materials provided with the distribution.\n')
f.write(' *     * Neither the name of Willow Garage, Inc. nor the names of its\n')
f.write(' *       contributors may be used to endorse or promote products derived from\n')
f.write(' *       this software without specific prior written permission.\n')
f.write(' *\n')
f.write(' * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"\n')
f.write(' * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE\n')
f.write(' * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE\n')
f.write(' * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE\n')
f.write(' * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR\n')
f.write(' * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF\n')
f.write(' * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS\n')
f.write(' * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN\n')
f.write(' * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)\n')
f.write(' * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE\n')
f.write(' * POSSIBILITY OF SUCH DAMAGE.\n')
f.write(' */\n\n')

add_macro(f, "DEBUG", "Debug")
add_macro(f, "INFO", "Info")
add_macro(f, "WARN", "Warn")
add_macro(f, "ERROR", "Error")
add_macro(f, "FATAL", "Fatal")




