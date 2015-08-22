set(module px4_simple_app)
set(stack 1024)
set(main px4_simple_app_main)
add_library(${module} STATIC EXCLUDE_FROM_ALL
	px4_simple_app.c
	)

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
