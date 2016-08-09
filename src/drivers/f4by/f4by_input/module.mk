#
# Interface driver for the PX4FMU board
#

MODULE_COMMAND	 = f4by_input
SRCS		 = 	f4by_input.cpp \
				sbus.c \
				dsm.c
