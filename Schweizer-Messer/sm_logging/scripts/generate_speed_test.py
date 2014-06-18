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
roslib.load_manifest('rosconsole')

base_path = roslib.packages.get_pkg_dir('rosconsole')

f = open('%s/test/speed_test.cpp' % (base_path), 'w')

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

f.write('#include "ros/console.h"\n')
f.write('#include "log4cxx/appenderskeleton.h"\n')

#for i in range(0,int(sys.argv[1])):
#    f.write('void info%s(int i) { ROS_INFO("Info%s: %%d", i); }\n' %(i,i))
#    f.write('void warn%s(int i) { ROS_WARN("Warn%s: %%d", i); }\n' %(i,i))
#    f.write('void error%s(int i) { ROS_ERROR("Error%s: %%d", i); }\n' %(i,i))
#    f.write('void debug%s(int i) { ROS_DEBUG("Debug%s: %%d", i); }\n' %(i,i))
#    f.write('void errorr%s(int i) { ROS_ERROR("Error2%s: %%d", i); }\n' %(i,i))

f.write('class NullAppender : public log4cxx::AppenderSkeleton {\n')
f.write('protected:\n')
f.write('virtual void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool){printf("blah\\n");}\n')
f.write('virtual void close() {}\n')
f.write('virtual bool requiresLayout() const { return false; } };\n')

f.write('int main(int argc, char** argv)\n{\n')
f.write('ROSCONSOLE_AUTOINIT; \nlog4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->removeAllAppenders();\n')
f.write('log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->addAppender(new NullAppender);\n')
f.write('log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(log4cxx::Level::getFatal());\n')
f.write('for (int i = 0;i < %s; ++i)\n{\n' %(sys.argv[2]))

for i in range(0,int(sys.argv[1])):
    #f.write('info%s(i);\n' %(i))
    #f.write('warn%s(i);\n' %(i))
    #f.write('error%s(i);\n' %(i))
    #f.write('debug%s(i);\n' %(i))
    #f.write('errorr%s(i);\n' %(i))
    f.write('ROS_INFO("test");')
    f.write('ROS_WARN("test");')
    f.write('ROS_ERROR("test");')
    f.write('ROS_DEBUG("test");')
    f.write('ROS_ERROR("test");')

f.write('}\n')
f.write('}\n')

