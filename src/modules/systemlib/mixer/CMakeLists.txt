set(module mixerlib)
set(stack)
set(main)

set(BIN ${CMAKE_CURRENT_BINARY_DIR})
set(SRC ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_command(OUTPUT ${BIN}/mixer_multirotor.generated.h
	COMMAND PYTHONPATH=${PYTHONPATH} ${PYTHON_EXECUTABLE} 
		./multi_tables.py > ${BIN}/mixer_multirotor.generated.h
	WORKING_DIRECTORY ${SRC}
	)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(SRCS
	${CMAKE_CURRENT_BINARY_DIR}/mixer_multirotor.generated.h
	mixer.cpp
	mixer_group.cpp
	mixer_multirotor.cpp
	mixer_simple.cpp
	mixer_load.c
	)
add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
