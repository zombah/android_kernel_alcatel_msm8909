#
# Makefile for nfc devices
#

obj-$(CONFIG_BCM2079X_NFC)	+= bcm2079x-i2c.o #[BUFFIX]-Add begin - by TCTNB.XQJ, FR-901585, 2015/1/26,add NFC driver
obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
