#
# Makefile for the Foo *default* configuration
#

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= modules/sensors
MODULES		+= drivers/mpu9x50

#
# System commands
#
MODULES	+= systemcmds/param


#
# General system control
#
#MODULES		+= modules/mavlink

#
# Estimation modules (EKF/ SO3 / other filters)
#

#
# Vehicle Control
#

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB
#MODULES		+= modules/dataman

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion

#
# QuRT port
#
MODULES		+= platforms/qurt/px4_layer

#
# Unit tests
#

#
# sources for muorb over fastrpc
#
MODULES         += modules/muorb/adsp/
