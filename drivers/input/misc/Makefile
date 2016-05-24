#
# Makefile for the input misc drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT_88PM860X_ONKEY)	+= 88pm860x_onkey.o
obj-$(CONFIG_INPUT_88PM80X_ONKEY)	+= 88pm80x_onkey.o
obj-$(CONFIG_INPUT_AB8500_PONKEY)	+= ab8500-ponkey.o
obj-$(CONFIG_INPUT_AD714X)		+= ad714x.o
obj-$(CONFIG_INPUT_AD714X_I2C)		+= ad714x-i2c.o
obj-$(CONFIG_INPUT_AD714X_SPI)		+= ad714x-spi.o
obj-$(CONFIG_INPUT_ADXL34X)		+= adxl34x.o
obj-$(CONFIG_INPUT_ADXL34X_I2C)		+= adxl34x-i2c.o
obj-$(CONFIG_INPUT_ADXL34X_SPI)		+= adxl34x-spi.o
obj-$(CONFIG_INPUT_APANEL)		+= apanel.o
obj-$(CONFIG_INPUT_ARIZONA_HAPTICS)	+= arizona-haptics.o
obj-$(CONFIG_INPUT_ATI_REMOTE2)		+= ati_remote2.o
obj-$(CONFIG_INPUT_ATLAS_BTNS)		+= atlas_btns.o
obj-$(CONFIG_INPUT_BFIN_ROTARY)		+= bfin_rotary.o
obj-$(CONFIG_INPUT_BMA150)		+= bma150.o
obj-$(CONFIG_INPUT_CM109)		+= cm109.o
obj-$(CONFIG_INPUT_CMA3000)		+= cma3000_d0x.o
obj-$(CONFIG_INPUT_CMA3000_I2C)		+= cma3000_d0x_i2c.o
obj-$(CONFIG_INPUT_COBALT_BTNS)		+= cobalt_btns.o
obj-$(CONFIG_INPUT_DA9052_ONKEY)	+= da9052_onkey.o
obj-$(CONFIG_INPUT_DA9055_ONKEY)	+= da9055_onkey.o
obj-$(CONFIG_INPUT_DM355EVM)		+= dm355evm_keys.o
obj-$(CONFIG_INPUT_GP2A)		+= gp2ap002a00f.o
obj-$(CONFIG_INPUT_GPIO_TILT_POLLED)	+= gpio_tilt_polled.o
obj-$(CONFIG_INPUT_GPIO)		+= gpio_event.o gpio_matrix.o gpio_input.o gpio_output.o gpio_axis.o
obj-$(CONFIG_INPUT_HBTP_INPUT)		+= hbtp_input.o hbtp_vm.o
obj-$(CONFIG_HP_SDC_RTC)		+= hp_sdc_rtc.o
obj-$(CONFIG_INPUT_IMS_PCU)		+= ims-pcu.o
obj-$(CONFIG_INPUT_IXP4XX_BEEPER)	+= ixp4xx-beeper.o
obj-$(CONFIG_INPUT_KEYCHORD)		+= keychord.o
obj-$(CONFIG_INPUT_KEYSPAN_REMOTE)	+= keyspan_remote.o
obj-$(CONFIG_INPUT_KXTJ9)		+= kxtj9.o
obj-$(CONFIG_INPUT_M68K_BEEP)		+= m68kspkr.o
obj-$(CONFIG_INPUT_MAX8925_ONKEY)	+= max8925_onkey.o
obj-$(CONFIG_INPUT_MAX8997_HAPTIC)	+= max8997_haptic.o
obj-$(CONFIG_INPUT_MC13783_PWRBUTTON)	+= mc13783-pwrbutton.o
obj-$(CONFIG_INPUT_MMA8450)		+= mma8450.o
obj-$(CONFIG_INPUT_MPU3050)		+= mpu3050.o
obj-$(CONFIG_INPUT_PCAP)		+= pcap_keys.o
obj-$(CONFIG_INPUT_PCF50633_PMU)	+= pcf50633-input.o
obj-$(CONFIG_INPUT_PCF8574)		+= pcf8574_keypad.o
obj-$(CONFIG_INPUT_PCSPKR)		+= pcspkr.o
obj-$(CONFIG_INPUT_PM8XXX_VIBRATOR)	+= pm8xxx-vibrator.o
obj-$(CONFIG_INPUT_PMIC8XXX_PWRKEY)	+= pmic8xxx-pwrkey.o
obj-$(CONFIG_INPUT_POWERMATE)		+= powermate.o
obj-$(CONFIG_INPUT_PWM_BEEPER)		+= pwm-beeper.o
obj-$(CONFIG_INPUT_RB532_BUTTON)	+= rb532_button.o
obj-$(CONFIG_INPUT_RETU_PWRBUTTON)	+= retu-pwrbutton.o
obj-$(CONFIG_INPUT_GPIO_ROTARY_ENCODER)	+= rotary_encoder.o
obj-$(CONFIG_INPUT_SGI_BTNS)		+= sgi_btns.o
obj-$(CONFIG_INPUT_SPARCSPKR)		+= sparcspkr.o
obj-$(CONFIG_INPUT_TWL4030_PWRBUTTON)	+= twl4030-pwrbutton.o
obj-$(CONFIG_INPUT_TWL4030_VIBRA)	+= twl4030-vibra.o
obj-$(CONFIG_INPUT_TWL6040_VIBRA)	+= twl6040-vibra.o
obj-$(CONFIG_INPUT_UINPUT)		+= uinput.o
obj-$(CONFIG_INPUT_WISTRON_BTNS)	+= wistron_btns.o
obj-$(CONFIG_INPUT_WM831X_ON)		+= wm831x-on.o
obj-$(CONFIG_INPUT_XEN_KBDDEV_FRONTEND)	+= xen-kbdfront.o
obj-$(CONFIG_INPUT_YEALINK)		+= yealink.o
obj-$(CONFIG_BMP18X)			+= bmp18x-core.o
obj-$(CONFIG_BMP18X_I2C)		+= bmp18x-i2c.o
obj-$(CONFIG_SENSORS_MMC3416X)		+= mmc3416x.o
obj-$(CONFIG_SENSORS_MMA8X5X)	  	+= mma8x5x.o
# obj-$(CONFIG_SENSORS_STK3X1X)		+= stk3x1x.o
obj-$(CONFIG_SENSORS_CAPELLA_CM36283)	+= cm36283.o
obj-$(CONFIG_SENSORS_MPU6050)		+= mpu6050.o
obj-$(CONFIG_SENSORS_AKM8963)		+= akm8963.o
# obj-$(CONFIG_SENSORS_AKM09911)		+= akm09911.o
# obj-$(CONFIG_SENSORS_LIS3DH)		+= lis3dh_acc.o
obj-$(CONFIG_SENSORS_AP3426)		+= ap3426.o
# obj-$(CONFIG_SENSORS_BMA2X2)	+= bstclass.o

# obj-$(CONFIG_SENSORS_BMA2X2)	+= bma2x2.o
# obj-$(CONFIG_SENSORS_LTR553)	+= ltr553.o

# ifeq ($(CONFIG_SENSORS_BMA2X2_ENABLE_INT1),y)
# 	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT1
# endif

# ifeq ($(CONFIG_BOSCH_BMA2X2_ENABLE_INT2),y)
# 	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2
# endif

ifdef CONFIG_TCT_8909_PIXI35
obj-$(CONFIG_SENSORS)       += pixi35/
endif

ifdef CONFIG_TCT_8909_PIXI355
obj-$(CONFIG_SENSORS)       += pixi355/
endif

# + Cedar.Tan add for product pixi355_tmo,20150521
ifdef CONFIG_TCT_8909_PIXI355_TMO
obj-$(CONFIG_SENSORS)       += pixi355_tmo/
endif
# - Cedar.Tan add for product pixi355_tmo,20150521

# + yijun.chen add for product pixi445,20150720
ifdef CONFIG_TCT_8909_PIXI445_TF
obj-$(CONFIG_SENSORS)       += pixi445_tf/
endif
# - yijun.chen add for product pixi445,20150720

#[PLATFORM]-Add-BEGIN by TCTNB.ZXZ, RR-876380, 2015/01/03, add stk3311 sensor for pop3
ifdef CONFIG_TCT_8909_POP3
obj-$(CONFIG_SENSORS_STK3X1X_TCT)       += stk3x1x_tct/
endif
#[PLATFORM]-Add-END by TCTNB.ZXZ
#[PLATFORM]-Add by TCTNB.ZXZ, RR-966458, 2015/04/02, add KXTJ2-1009 Gsensor
obj-$(CONFIG_SENSORS_KXTJ9_TCT)       += kxtj9_tct/
#[PLATFORM]-Add-END by TCTNB.ZXZ

#[PLATFORM]-Add-BEGIN by TCTNB.ZXZ, RR-876617, 2015/01/04, add BMA223 sensor for pop3
ifdef CONFIG_TCT_8909_POP3
obj-$(CONFIG_SENSORS_BMA2X2_TCT)       += bma2x2_tct/

ifeq ($(CONFIG_SENSORS_BMA2X2_ENABLE_INT1_TCT),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT1
endif

ifeq ($(CONFIG_BOSCH_BMA2X2_ENABLE_INT2_TCT),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2
endif

endif
#[PLATFORM]-Add-END by TCTNB.ZXZ

#[PLATFORM]-Add-BEGIN by TCTNB.ZXZ, FR-966665, 2015/04/27, add ADUX1050 sensor
obj-$(CONFIG_SENSORS_PSENSOR)   += audx1050/
#[PLATFORM]-Add-END by TCTNB.ZXZ


ifdef CONFIG_TCT_8909_PIXI384G
obj-$(CONFIG_SENSORS_BMA2X2_TCT)       += bma2x2_tct/
obj-$(CONFIG_SENSORS_HALL)              += hall/
endif
