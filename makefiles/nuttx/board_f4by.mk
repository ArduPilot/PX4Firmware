#
# Board-specific definitions for the F4BY
#

#
# Configure the toolchain
#
CONFIG_ARCH			 		= CORTEXM4F
CONFIG_BOARD			 	= F4BY

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
