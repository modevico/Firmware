#
# Common OS porting APIs
#

SRCS		 = px4_getopt.c 

ifneq ($(PX4_TARGET_OS),qurt)
SRCS		+= px4_log.c
endif
