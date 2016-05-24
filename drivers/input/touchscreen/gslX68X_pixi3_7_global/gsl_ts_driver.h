/* drivers/input/touchscreen/gslX68X.h
 * 
 * 2010 - 2013 SLIEAD Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the SLIEAD's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */

//#include <mach/ldo.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
//#define GSL_PROXIMITY_SENSOR
#ifdef GSL_PROXIMITY_SENSOR
#include <linux/sensors.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include "gsl_ts_8074.h"
//#include "../ts_func_test.h"

#define GSL_DEBUG 
//#define GSL_TIMER					//定时器开关
#define TPD_PROC_DEBUG				//proc文件系统接口红开关
#define GSL9XX_VDDIO_1800 0
#define GSL_REPORT_POINT_SLOT

//#define GSL_GESTURE
//#define GSL_COMPATIBLE_GPIO
#define GSL_ALG_ID

/*define i2c addr and device name*/
#define GSL_TS_ADDR 				0x40
#define GSL_TS_NAME				"GSL_TP"

/*define irq and reset gpio num*/
#define GSL_RST_GPIO_NUM	0
#define GSL_IRQ_GPIO_NUM	1
#define GSL_IRQ_NUM			gpio_to_irq(GSL_IRQ_GPIO_NUM)
//#define GSL_IRQ_NUM		sprd_alloc_gpio_irq(GSL_IRQ_GPIO_NUM)
#define GSL_IRQ_NAME		"gsl_irq"
#define GSL_RST_NAME		"gsl_reset"

#define GSL_COORDS_ARR_SIZE	4
#define GSL_VTG_MIN_UV		2600000
#define GSL_VTG_MAX_UV		3300000
#define GSL_I2C_VTG_MIN_UV	1800000
#define GSL_I2C_VTG_MAX_UV	1800000
#define MAX_BUTTONS			4

#if 0
/*vdd power*/
#define GSL_POWER_ON()	do{\
			}while(0)
#endif		
#define GSL_TIMER_CHECK_CIRCLE		200
#define GSL_PRESSURE				50

/*debug of time*/
#define TPD_DEBUG_TIME				0x20141209

/*define screen of resolution ratio*/
#define GSL_MAX_X		600
#define GSL_MAX_Y		1024

/*i2c of adapter number*/

/*virtual keys*/
//#define TOUCH_VIRTUAL_KEYS

/*button of key*/
#define GSL_HAVE_TOUCH_KEY 			1
#if GSL_HAVE_TOUCH_KEY
	struct key_data{
		u16 key;
		u32 x_min;
		u32 x_max;
		u32 y_min;
		u32 y_max;
	};
	#define GSL_KEY_NUM	 3	 
	struct key_data gsl_key_data[GSL_KEY_NUM] = {
		{KEY_MENU,50,100,806,846},
		{KEY_HOMEPAGE,200,250,806,846},
		{KEY_BACK,370,420,806,846},
	};
#endif

struct gsl_touch_info{
	int x[10];
	int y[10];
	int id[10];
	int finger_num;
};

struct gsl_ts_platform_data {
	const char *name;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 num_max_touches;
	u32 button_map[MAX_BUTTONS];
	u32 num_buttons;
	u32 hard_reset_delay_ms;
	u32 post_hard_reset_delay_ms;
};

struct gsl_ts_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct mutex gsl_i2c_lock;
	struct gsl_ts_platform_data *pdata;
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct work_struct fb_notify_work;
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend pm;
	#endif

	struct gsl_touch_info *cinfo;
	bool suspended;							//0 normal;1 the machine is suspend;
	bool updating_fw;						//0 normal;1 download the firmware;
	u32 gsl_up_flag;						//0 normal;1 have one up event;
	u32 gsl_point_state;					//the point down and up of state	
#ifdef GSL_TIMER
	struct delayed_work		timer_work;
	struct workqueue_struct	*timer_wq;
	volatile int gsl_timer_flag;			//0:first test	1:second test 2:doing gsl_load_fw
	unsigned int gsl_timer_data;		
#endif
#if GSL_HAVE_TOUCH_KEY
	int gsl_key_state;
#endif
#ifdef GSL_PROXIMITY_SENSOR
	struct input_dev *input_dev_ps;
	struct sensors_classdev ps_cdev;
#endif
	struct work_struct resume_work;
	u8 fw_version;
	u8 pannel_id;
//	struct ts_func_test_device ts_test_dev;
};

/* Print Information */
#ifdef GSL_DEBUG
#undef dev_info
#define dev_info(fmt, args...)   \
        do{                              \
                printk("[tp-gsl][%s]"fmt,__func__, ##args);     \
        }while(0)
#else
#define dev_info(fmt, args...)   //
#endif
static unsigned char gsl_cfg_index = 0;

#ifdef GSL_PROXIMITY_SENSOR
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "proximity",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

#ifdef GSL_ALG_ID
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);
extern unsigned int gsl_mask_tiaoping(void);
extern int gsl_obtain_gesture(void);
extern void gsl_FunIICRead(unsigned int (*fun) (unsigned int *,unsigned int,unsigned int));

#endif

static struct fw_config_type gsl_cfg_table[9] = {
/*0*/{GSLX68X_FW_DJ,(sizeof(GSLX68X_FW_DJ)/sizeof(struct fw_data)),
	gsl_config_data_id_DJ,(sizeof(gsl_config_data_id_DJ)/4)},
///*1*/{GSLX68X_FW_CS,(sizeof(GSLX68X_FW_CS)/sizeof(struct fw_data)),
//	gsl_config_data_id_CS,(sizeof(gsl_config_data_id_CS)/4)},
/*2*/{NULL,0,NULL,0},
/*3*/{NULL,0,NULL,0},
/*4*/{NULL,0,NULL,0},
/*5*/{NULL,0,NULL,0},
/*6*/{NULL,0,NULL,0},
/*7*/{NULL,0,NULL,0},
/*8*/{NULL,0,NULL,0},

};
