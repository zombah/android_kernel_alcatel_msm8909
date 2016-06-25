#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_TOUCHSCREEN_FT5X06)	+= ft5x06_ts.o ft_rawdata.a

#+ cedar add for forced upgrading of tp firmware
ifeq ($(TARGET_BUILD_MMITEST), true)
KBUILD_CFLAGS += -DMINI_MODE_TO_CTP
endif
#- cedar add for forced upgrading of tp firmware
