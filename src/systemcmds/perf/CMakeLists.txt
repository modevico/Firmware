set(module perf)
set(stack 1800)
set(main perf)
add_library(${module} STATIC EXCLUDE_FROM_ALL
	perf.c
	)
set_target_properties(${modules} PROPERTIES
	COMPILE_FLAGS "-Os")
add_dependencies(${module} systemlib)

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
