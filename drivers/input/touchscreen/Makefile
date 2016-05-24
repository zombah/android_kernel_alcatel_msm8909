#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

wm97xx-ts-y := wm97xx-core.o

obj-$(CONFIG_TOUCHSCREEN_88PM860X)	+= 88pm860x-ts.o
obj-$(CONFIG_TOUCHSCREEN_AD7877)	+= ad7877.o
obj-$(CONFIG_TOUCHSCREEN_AD7879)	+= ad7879.o
obj-$(CONFIG_TOUCHSCREEN_AD7879_I2C)	+= ad7879-i2c.o
obj-$(CONFIG_TOUCHSCREEN_AD7879_SPI)	+= ad7879-spi.o
obj-$(CONFIG_TOUCHSCREEN_ADS7846)	+= ads7846.o
obj-$(CONFIG_TOUCHSCREEN_ATMEL_MXT)	+= atmel_mxt_ts.o
obj-$(CONFIG_TOUCHSCREEN_ATMEL_MAXTOUCH_TS)	+= atmel_maxtouch_ts.o
obj-$(CONFIG_TOUCHSCREEN_ATMEL_TSADCC)	+= atmel_tsadcc.o
obj-$(CONFIG_TOUCHSCREEN_AUO_PIXCIR)	+= auo-pixcir-ts.o
obj-$(CONFIG_TOUCHSCREEN_BU21013)	+= bu21013_ts.o
obj-$(CONFIG_TOUCHSCREEN_CY8CTMG110)	+= cy8ctmg110_ts.o
obj-$(CONFIG_TOUCHSCREEN_CYTTSP_CORE)	+= cyttsp_core.o
obj-$(CONFIG_TOUCHSCREEN_CYTTSP_I2C)	+= cyttsp_i2c.o
obj-$(CONFIG_TOUCHSCREEN_CYTTSP_SPI)	+= cyttsp_spi.o
obj-$(CONFIG_TOUCHSCREEN_DA9034)	+= da9034-ts.o
obj-$(CONFIG_TOUCHSCREEN_DA9052)	+= da9052_tsi.o
obj-$(CONFIG_TOUCHSCREEN_DYNAPRO)	+= dynapro.o
obj-$(CONFIG_TOUCHSCREEN_EDT_FT5X06)	+= edt-ft5x06.o
obj-$(CONFIG_TOUCHSCREEN_HAMPSHIRE)	+= hampshire.o
obj-$(CONFIG_TOUCHSCREEN_GUNZE)		+= gunze.o
obj-$(CONFIG_TOUCHSCREEN_EETI)		+= eeti_ts.o
obj-$(CONFIG_TOUCHSCREEN_ELO)		+= elo.o
obj-$(CONFIG_TOUCHSCREEN_EGALAX)	+= egalax_ts.o
obj-$(CONFIG_TOUCHSCREEN_FUJITSU)	+= fujitsu_ts.o
obj-$(CONFIG_TOUCHSCREEN_GEN_VKEYS)	+= gen_vkeys.o
obj-$(CONFIG_TOUCHSCREEN_ILI210X)	+= ili210x.o
obj-$(CONFIG_TOUCHSCREEN_INEXIO)	+= inexio.o
obj-$(CONFIG_TOUCHSCREEN_INTEL_MID)	+= intel-mid-touch.o
obj-$(CONFIG_TOUCHSCREEN_LPC32XX)	+= lpc32xx_ts.o
obj-$(CONFIG_TOUCHSCREEN_MAX11801)	+= max11801_ts.o
obj-$(CONFIG_TOUCHSCREEN_MC13783)	+= mc13783_ts.o
obj-$(CONFIG_TOUCHSCREEN_MCS5000)	+= mcs5000_ts.o
obj-$(CONFIG_TOUCHSCREEN_MIGOR)		+= migor_ts.o
obj-$(CONFIG_TOUCHSCREEN_MMS114)	+= mms114.o
obj-$(CONFIG_TOUCHSCREEN_MTOUCH)	+= mtouch.o
obj-$(CONFIG_TOUCHSCREEN_MK712)		+= mk712.o
obj-$(CONFIG_TOUCHSCREEN_HP600)		+= hp680_ts_input.o
obj-$(CONFIG_TOUCHSCREEN_HP7XX)		+= jornada720_ts.o
obj-$(CONFIG_TOUCHSCREEN_HTCPEN)	+= htcpen.o
obj-$(CONFIG_TOUCHSCREEN_USB_COMPOSITE)	+= usbtouchscreen.o
obj-$(CONFIG_TOUCHSCREEN_PCAP)		+= pcap_ts.o
obj-$(CONFIG_TOUCHSCREEN_PENMOUNT)	+= penmount.o
obj-$(CONFIG_TOUCHSCREEN_PIXCIR)	+= pixcir_i2c_ts.o
obj-$(CONFIG_TOUCHSCREEN_S3C2410)	+= s3c2410_ts.o
obj-$(CONFIG_TOUCHSCREEN_ST1232)	+= st1232.o
obj-$(CONFIG_TOUCHSCREEN_STMPE)		+= stmpe-ts.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_v21) += synaptics_dsx/
obj-$(CONFIG_TOUCHSCREEN_TI_AM335X_TSC)	+= ti_am335x_tsc.o
obj-$(CONFIG_TOUCHSCREEN_TNETV107X)	+= tnetv107x-ts.o
obj-$(CONFIG_TOUCHSCREEN_TOUCHIT213)	+= touchit213.o
obj-$(CONFIG_TOUCHSCREEN_TOUCHRIGHT)	+= touchright.o
obj-$(CONFIG_TOUCHSCREEN_TOUCHWIN)	+= touchwin.o
obj-$(CONFIG_TOUCHSCREEN_TSC_SERIO)	+= tsc40.o
obj-$(CONFIG_TOUCHSCREEN_TSC2005)	+= tsc2005.o
obj-$(CONFIG_TOUCHSCREEN_TSC2007)	+= tsc2007.o
obj-$(CONFIG_TOUCHSCREEN_UCB1400)	+= ucb1400_ts.o
obj-$(CONFIG_TOUCHSCREEN_WACOM_W8001)	+= wacom_w8001.o
obj-$(CONFIG_TOUCHSCREEN_WACOM_I2C)	+= wacom_i2c.o
obj-$(CONFIG_TOUCHSCREEN_WM831X)	+= wm831x-ts.o
obj-$(CONFIG_TOUCHSCREEN_WM97XX)	+= wm97xx-ts.o
wm97xx-ts-$(CONFIG_TOUCHSCREEN_WM9705)	+= wm9705.o
wm97xx-ts-$(CONFIG_TOUCHSCREEN_WM9712)	+= wm9712.o
wm97xx-ts-$(CONFIG_TOUCHSCREEN_WM9713)	+= wm9713.o
obj-$(CONFIG_TOUCHSCREEN_WM97XX_ATMEL)	+= atmel-wm97xx.o
obj-$(CONFIG_TOUCHSCREEN_WM97XX_MAINSTONE)	+= mainstone-wm97xx.o
obj-$(CONFIG_TOUCHSCREEN_WM97XX_ZYLONITE)	+= zylonite-wm97xx.o
obj-$(CONFIG_TOUCHSCREEN_W90X900)	+= w90p910_ts.o
obj-$(CONFIG_TOUCHSCREEN_TPS6507X)	+= tps6507x-ts.o
# modify(add) by bin.su.nb for [TP driver development (1st source)] with PR876205 begin . 20150103
ifdef CONFIG_TCT_8909_POP3
obj-$(CONFIG_TOUCHSCREEN_FT5436)	+= tp_FT5436/
endif
# modify(add) by bin.su.nb for [TP driver development (1st source)] with PR876205 end
obj-$(CONFIG_TOUCHSCREEN_MSTAR21XX)	+= msg21xx_ts.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI4_DEV)	+= synaptics_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) 	+= synaptics_fw_update.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)		+= synaptics_i2c_rmi4.o
obj-$(CONFIG_TOUCHSCREEN_GT9XX)		+= gt9xx/
obj-$(CONFIG_TOUCHSCREEN_BU21150)	+= bu21150.o
obj-$(CONFIG_INPUT_MT_WRAPPER)		+= input_mt_wrapper.o

ifdef CONFIG_TCT_8909_PIXI35
obj-$(CONFIG_INPUT_TOUCHSCREEN)       += pixi35/
# add by mike.li for 2st src msg2238 begin. [PRxxxx][2015.03.06]
obj-$(CONFIG_INPUT_TOUCHSCREEN)       += msg2238_pixi35/
# add by mike.li for 2st src msg2238 end. [PRxxxx][2015.03.06]
endif

ifdef CONFIG_TCT_8909_PIXI355
#Add by Luoxingxing for gt9157 -START- [2015.04.17]
obj-$(CONFIG_INPUT_TOUCHSCREEN)       += gt9157_pixi355/
obj-$(CONFIG_INPUT_TOUCHSCREEN)       += s2333_pixi355/
#Add by Luoxingxing for gt9157 -END- [2015.04.17]
endif

# liyifan start, add TMO Micro CTP driver control,20150520
ifdef CONFIG_TCT_8909_PIXI355_TMO
obj-$(CONFIG_INPUT_TOUCHSCREEN)       += gt9157_pixi355/
endif
# liyifan end, add TMO Micro CTP control

#add by ke.li for pixi445_tf ft6336
ifdef CONFIG_TCT_8909_PIXI445_TF
#obj-$(CONFIG_FT6336_TOUCHPANEL)        += ft6336_pixi445_tf/
obj-$(CONFIG_INPUT_TOUCHSCREEN)        += msg2238_pixi445_tf/
endif
#add by ke.li end

# modify(add) by wei.qiu for [TP driver development] with FR966443 begin . 20150402
# modify(add) by bin.su for [TP  2nd TP driver development ] with FR1013959 begin . 20150605
ifdef CONFIG_TCT_8909_PIXI37
obj-$(CONFIG_TOUCHSCREEN_GT912)		+= gt912_pixi3_7_global/
obj-$(CONFIG_GSL_TOUCHPANEL_DRIVER)	+= gslX68X_pixi3_7_global/
else
ifdef CONFIG_TCT_8909_PIXI384G
obj-$(CONFIG_TOUCHSCREEN_GT912)		+= gt912_pixi3_8_global/
else
obj-$(CONFIG_TOUCHSCREEN_GT9XX)		+= gt9xx/
endif
endif
#modify(add) end.
# modify(add) by wei.qiu for [TP driver development] with FR966443 end . 20150402




