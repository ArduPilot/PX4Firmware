#
# Makefile for the px4fmu-v4_APM configuration
#
include $(SKETCHBOOK)/mk/F4BY/f4by_common.mk

#MODULES		+= drivers/pwm_input
#MODULES     += modules/uavcan
MODULES     += lib/mathlib
MODULES		+= drivers/boards/f4by
