#
# Copyright (C) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#	notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#	notice, this list of conditions and the following disclaimer in
#	the documentation and/or other materials provided with the
#	distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#	used to endorse or promote products derived from this software
#	without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

include(CMakeForceCompiler)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(px4_utils)

add_definitions(
	-D_PID_T -D_UID_T -D_TIMER_T
	-Drestrict=
	-D_DEBUG
	-Wno-error=shadow
	)

# optimisation flags
#
set(ARCHOPTIMIZATION
	-O0
	-g
	-fno-strict-aliasing
	-fdata-sections
	-fpic
	-fno-zero-initialized-in-bss
	)

# Language-specific flags
#
set(ARCHCFLAGS
	-std=gnu99
	-D__CUSTOM_FILE_IO__
	)

set(ARCHCXXFLAGS
	-fno-exceptions
	-fno-rtti
	-std=c++11
	-fno-threadsafe-statics
	-DCONFIG_WCHAR_BUILTIN
	-D__CUSTOM_FILE_IO__
	)

set(ARCHWARNINGS
	-Wall
	-Wextra
	-Werror
	-Wno-unused-parameter
	-Wno-unused-function
	-Wno-unused-variable
	-Wno-cast-align
	-Wno-missing-braces
	-Wno-strict-aliasing
#   -Werror=float-conversion - works, just needs to be phased in with some effort and needs GCC 4.9+
#   -Wcast-qual  - generates spurious noreturn attribute warnings, try again later
#   -Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
#   -Wcast-align - would help catch bad casts in some cases, but generates too many false positives
	)

# C-specific warnings
#
set(ARCHCWARNINGS
	${ARCHWARNINGS}
	-Wstrict-prototypes
	-Wnested-externs
	)

# C++-specific warnings
#
set(ARCHWARNINGSXX
	${ARCHWARNINGS}
	-Wno-missing-field-initializers
	)

# Flags we pass to the C compiler
#
list2string(CFLAGS 
	${ARCHCFLAGS}
	${ARCHCWARNINGS}
	${ARCHOPTIMIZATION}
	${ARCHCPUFLAGS}
	${ARCHINCLUDES}
	${INSTRUMENTATIONDEFINES}
	${ARCHDEFINES}
	${EXTRADEFINES}
	${EXTRACFLAGS}
	)

# Flags we pass to the C++ compiler
#
list2string(CXXFLAGS
	${ARCHCXXFLAGS}
	${ARCHWARNINGSXX}
	${ARCHOPTIMIZATION}
	${ARCHCPUFLAGS}
	${ARCHXXINCLUDES}
	${INSTRUMENTATIONDEFINES}
	${ARCHDEFINES}
	${EXTRADEFINES}
	${EXTRACXXFLAGS}
	${HEXAGON_INCLUDE_DIRS}
	)

# Flags we pass to the assembler
#
list2string(AFLAGS
	${CFLAGS} 
	-D__ASSEMBLY__
	${EXTRADEFINES}
	${EXTRAAFLAGS}
	)

# Set cmake flags
#
list2string(CMAKE_C_FLAGS
	${CMAKE_C_FLAGS}
	${CFLAGS}
	)

set(POSIX_CMAKE_C_FLAGS ${CMAKE_C_FLAGS} CACHE STRING "cflags")

list2string(CMAKE_CXX_FLAGS
	${CMAKE_CXX_FLAGS}
	${CXXFLAGS}
	)

set(POSIX_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "cxxflags")

