set(module led)
set(stack)
set(main)
add_library(${module} STATIC EXCLUDE_FROM_ALL
	led.cpp
	)
set_target_properties(${module} PROPERTIES
	COMPILE_FLAGS "-Os")

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
