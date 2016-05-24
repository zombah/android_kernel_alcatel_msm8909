#
# Makefile for the input misc drivers.
#

obj-$(CONFIG_SENSORS_BMG160)	+= bmg160_driver.o bmg160.o
obj-$(CONFIG_SENSORS_TSL277X)	+= tsl277x.o
obj-$(CONFIG_SENSORS_AKM09911)	+= akm09911.o

obj-$(CONFIG_SENSORS_BMA2X2)	+= bstclass.o
obj-$(CONFIG_SENSORS_BMA2X2)	+= bma2x2.o

ifeq ($(CONFIG_SENSORS_BMA2X2_ENABLE_INT1),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT1
endif

ifeq ($(CONFIG_BOSCH_BMA2X2_ENABLE_INT2),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2
endif
