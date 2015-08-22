set(module px4fmu-v2)
add_library(${module} STATIC EXCLUDE_FROM_ALL
	px4fmu_can.c
	px4fmu2_init.c
	px4fmu_pwm_servo.c
	px4fmu_spi.c
	px4fmu_usb.c
	px4fmu2_led.c
	)
set_target_properties(${module} PROPERTIES
	COMPILE_FLAGS -Os)

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
