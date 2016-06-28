#ifndef __TCT_USB_SPMI_H__
#define __TCT_USB_SPMI_H__

#if defined(CONFIG_TCT_SMB1360_AND_PM8909_CHARGER)
bool tct_usb_get_spmi_chg_option(void);
#else
static inline bool tct_usb_get_spmi_chg_option(void)
{
	return true;
}
#endif

#endif //__TCT_USB_SPMI_H__