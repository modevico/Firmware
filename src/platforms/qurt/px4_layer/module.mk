############################################################################
#
#   Copyright (c) 2014 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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
############################################################################

#
# NuttX / uORB adapter library
#

MODULE_NAME = dspal

SRCDIR=$(dir $(MODULE_MK))

SRCS		 = 	\
			px4_qurt_impl.cpp \
			px4_qurt_tasks.cpp  \
			hrt_thread.c \
                        hrt_queue.c \
                        hrt_work_cancel.c \
			../../posix/px4_layer/work_thread.c \
			../../posix/px4_layer/work_queue.c \
			../../posix/px4_layer/work_lock.c \
			../../posix/px4_layer/work_cancel.c \
			../../posix/px4_layer/lib_crc32.c \
			drv_hrt.c \
			../../posix/px4_layer/queue.c \
			../../posix/px4_layer/dq_addlast.c \
			../../posix/px4_layer/dq_remfirst.c \
			../../posix/px4_layer/sq_addlast.c \
			../../posix/px4_layer/sq_remfirst.c \
			../../posix/px4_layer/sq_addafter.c \
			../../posix/px4_layer/dq_rem.c \
                        qurt_stubs.c \
                        main.cpp
ifeq ($(CONFIG),qurt_hello)
SRCS +=			commands_hello.c
endif
ifeq ($(CONFIG),qurt_default)
SRCS +=			commands_default.c
endif
ifeq ($(CONFIG),qurt_muorb_test)
SRCS +=			commands_muorb_test.c
endif
ifeq ($(CONFIG),qurt_hil)
SRCS +=			commands_hil.c
endif
ifeq ($(CONFIG),qurt_adsp)
SRCS +=			commands_adsp.c
endif


MAXOPTIMIZATION	 = -Os
