#ifndef _PSENSOR_H_
#define _PSENSOR_H_


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/input/mt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

struct psensor_data {
	bool power_on;
	int irq_gpio;
	int enable_gpio;
	int p_sensor_value;
	struct regulator *vdd;
	struct regulator *vio;
	struct switch_dev *psensor_dev;
	struct work_struct psensor_work;
	struct workqueue_struct *psensor_wq;
	struct work_struct power_control_work;   //add by lizhi.wu@tcl.com 2014-11-24
	struct wake_lock psensor_wake_lock;	//add by lizhi.wu@tcl.com 2014-11-24
	struct work_struct usb_cali_work;	//when usb pull in or out, schedule this work
};

enum PSENSOR_STATUS{
	PSENSOR_STATUS_NEAR=0,
	PSENSOR_STATUS_FAR=1,
};

#if 0
struct psensor_driver_t{
	char *name;
	int (*init)(void);
};
#endif

extern u8 psensor_power_on;		//add by lizhi.wu@tcl.com 2015-4-6
void send_psensor_uevent(enum PSENSOR_STATUS val);

#define IQS_PINCTRL_STATE_DEACTIVE "iqs128_int_deactive"
#define IQS_PINCTRL_STATE_ACTIVE "iqs128_int_active"

struct iqs_pinctrl_info {
        struct pinctrl *pinctrl;
        struct pinctrl_state *gpio_state_active;
        struct pinctrl_state *gpio_state_deactive;
};
#endif
