set(module posix_px4_layer)
set(stack)
set(main)
set(SRCS
	px4_posix_impl.cpp
	px4_posix_tasks.cpp
	lib_crc32.c
	drv_hrt.c
	px4_log.c
	)
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
add_dependencies(${module} posix_px4_layer)
set_target_properties(${module} PROPERTIES COMPILE_FLAGS "-Os")

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
