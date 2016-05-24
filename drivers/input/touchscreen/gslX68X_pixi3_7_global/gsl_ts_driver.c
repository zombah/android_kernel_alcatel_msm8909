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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND) 
#include <linux/earlysuspend.h>
#endif
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h> 
#include <linux/of_gpio.h>

#include "gsl_ts_driver.h"

#ifdef GSL_REPORT_POINT_SLOT
    #include <linux/input/mt.h>
#endif
//subin #include <linux/productinfo.h>

/* Timer Function */
#ifdef GSL_TIMER
#define GSL_TIMER_CHECK_CIRCLE 200
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static char int_1st[4];
static char int_2nd[4];
#endif

/* Gesture Resume */
#ifdef GSL_GESTURE
typedef enum{
	GE_DISABLE = 0,
	GE_ENABLE = 1,
	GE_WAKEUP = 2,
	GE_NOWORK =3,
}GE_T;
static GE_T gsl_gesture_status = GE_DISABLE;
static volatile unsigned int gsl_gesture_flag = 1;
static char gsl_gesture_c = 0;
#endif

/* Proximity Sensor */
#ifdef GSL_PROXIMITY_SENSOR
#include <linux/wakelock.h>
#include <linux/sensors.h>
#endif

#if defined(GSL_PROXIMITY_SENSOR)
	static u8 bNeedResumeTp = 0;
#endif
#define CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#include <linux/sensors.h>
#include <linux/wakelock.h>
#define VPS_NAME "virtual-proximity"
//char flag_tp_down = 0;

#if 0
static struct virtualpsensor {
	char const *name;
	struct i2c_client *client;
	struct input_dev *virtualdevice;
	bool vps_enabled;
	struct sensors_classdev vps_cdev;
	bool virtual_proximity_data;
};
#endif 

struct virtualpsensor *gsl_vps;
int gsl_enabled;
struct input_dev *proximity_dev;
static u8 gsl_psensor_data[8]={0};
static struct sensors_classdev virtual_sensors_proximity_cdev = {
	.name = VPS_NAME,
	.vendor = "NULL",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "8.0",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL
};
static struct sensors_classdev vps_cdev ;
bool GSL_enable_irq_wake_flag = 0;
struct wake_lock GSL_wakelock;
#endif
	
/* Process for Android Debug Bridge */
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif


#define GSL_TEST_TP
#ifdef GSL_TEST_TP
extern void gsl_write_test_config(unsigned int cmd,int value);
extern unsigned int gsl_read_test_config(unsigned int cmd);
extern int gsl_obtain_array_data_ogv(short *ogv,int i_max,int j_max);
extern int gsl_obtain_array_data_dac(unsigned int *dac,int i_max,int j_max);
extern int gsl_tp_module_test(char *buf,int size);
#define GSL_PARENT_PROC_NAME "touchscreen"
#define GSL_OPENHSORT_PROC_NAME "ctp_openshort_test"
#define GSL_RAWDATA_PROC_NAME "ctp_rawdata"
#endif

/*define global variable*/
static struct gsl_ts_data *ts_data=NULL;

static void gsl_sw_init(struct i2c_client *client);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void gsl_early_suspend(struct early_suspend *handler);
static void gsl_early_resume(struct early_suspend *handler);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	return sprintf(buf,
         __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:900:40:40"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:900:40:40"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:900:40:40"
	 "\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.GSL_TP",
		.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void gsl_ts_virtual_keys_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	dev_info("%s\n",__func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
			&properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");
}

#endif
static int gsl_ts_read(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	int err = 0;
	u8 temp = reg;
	mutex_lock(&ts_data->gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (temp+8)&0x5c;
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
			temp = reg;
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
	}
	i2c_master_send(client,&reg,1);
	err = i2c_master_recv(client,&buf[0],num);
	mutex_unlock(&ts_data->gsl_i2c_lock);
	return (err == num)?1:-1;
}

#if 0
static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{

	int err = 0;
	u8 temp = reg;
	mutex_lock(&ts_data->gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (temp+8)&0x5c;
		err = i2c_master_send(client,&temp,1);
		if(err < 0) {
			goto err_i2c_transfer;
		}
		err = i2c_master_recv(client,&buf[0],4);
		if(err < 0) {
			goto err_i2c_transfer;
		}
		temp = reg;
		err = i2c_master_send(client,&temp,1);
		if(err < 0) {
			goto err_i2c_transfer;
		}
		err = i2c_master_recv(client,&buf[0],4);
		if(err < 0) {
			goto err_i2c_transfer;
		}
	}
	err = i2c_master_send(client,&reg,1);
	if(err < 0) {
		goto err_i2c_transfer;
	}
	err = i2c_master_recv(client,&buf[0],num);
	if(err != num) {
		err = -1;
		goto err_i2c_transfer;
	}
	mutex_unlock(&ts_data->gsl_i2c_lock);
	return 1;
err_i2c_transfer:
	mutex_unlock(&ts_data->gsl_i2c_lock);
	return err;
}
#endif

static int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	int err;
	u8 tmp_buf[num+1];
	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;

	mutex_lock(&ts_data->gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&ts_data->gsl_i2c_lock);

	return err;	
}

#ifdef GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;
	printk("tp-gsl-gesture %s\n",__func__);
	printk("gsl_read_oneframe_data:::addr=%x,len=%x\n",addr,len);
	for(i=0;i<len/2;i++){
		buf[0] = ((addr+i*8)/0x80)&0xff;
		buf[1] = (((addr+i*8)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*8)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*8)/0x80)>>24)&0xff;
		gsl_write_interface(ts_data->client,0xf0,buf,4);
		gsl_ts_read(ts_data->client,(addr+i*8)%0x80,(char *)&data[i*2],8);
	}
	if(len%2){
		buf[0] = ((addr+len*4 - 4)/0x80)&0xff;
		buf[1] = (((addr+len*4 - 4)/0x80)>>8)&0xff;
		buf[2] = (((addr+len*4 - 4)/0x80)>>16)&0xff;
		buf[3] = (((addr+len*4 - 4)/0x80)>>24)&0xff;
		gsl_write_interface(ts_data->client,0xf0,buf,4);
		gsl_ts_read(ts_data->client,(addr+len*4 - 4)%0x80,(char *)&data[len-1],4);
	}
	#if 1
	for(i=0;i<len;i++){
	printk("gsl_read_oneframe_data =%x\n",data[i]);	
	//printk("gsl_read_oneframe_data =%x\n",data[len-1]);
	}
	#endif
	
	return len;
}
#endif

static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
		gsl_write_interface(client, addr, buf, 4);	
	}
	printk("=============gsl_load_fw end==============\n");
}

static void gsl_io_control(struct i2c_client *client)
{
#if GSL9XX_VDDIO_1800
	u8 buf[4] = {0};
	int i;
	for(i=0;i<5;i++){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0xfe;
		buf[3] = 0x1;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0] = 0x5;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x80;
		gsl_write_interface(client,0x78,buf,4);
		msleep(5);
	}
	msleep(50);
#endif
}

static void gsl_start_core(struct i2c_client *client)
{
	//u8 tmp = 0x00;
	u8 buf[4] = {0};
	buf[0]=0;
	gsl_write_interface(client,0xe0,buf,4);
#ifdef GSL_ALG_ID
	{
	gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
	}
#endif	
}

static void gsl_reset_core(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	buf[0] = 0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	msleep(5);

	gsl_io_control(client);
}

static void gsl_clear_reg(struct i2c_client *client)
{
	u8 buf[4]={0};
	//clear reg
	buf[0]=0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	buf[0]=0x3;
	gsl_write_interface(client,0x80,buf,4);
	msleep(5);
	buf[0]=0x4;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	buf[0]=0x0;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	//clear reg

}

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
static void gsl_gain_psensor_data(struct i2c_client *client)
{
	u8 buf[4]={0};
	/**************************/
	buf[0]=0x3;
	gsl_write_interface(client,0xf0,buf,4);
	gsl_ts_read(client,0,&gsl_psensor_data[0],4);
	/**************************/
	
	buf[0]=0x4;
	gsl_write_interface(client,0xf0,buf,4);
	gsl_ts_read(client,0,&gsl_psensor_data[4],4);
	/**************************/
}
static int vps_set_enable(struct sensors_classdev *sensors_cdev,unsigned int enable)
{

	//u8 proximity_status[4] = {0};
	u8 ps_store_data[4]={0};
	gsl_enabled = enable ? 1 : 0;
	//DrvMainFirmwareProximityEnable(enable);
	if(enable == 1)
	{
#if 0
		gsl_ts_read(ts_data->client, 0xac,proximity_status,4);
		if(proximity_status[0] == 0x0)
		{
			input_report_abs(proximity_dev, ABS_DISTANCE, 1);
			input_sync(proximity_dev);
		}
		else if(proximity_status[0] == 0x1)
		{
			input_report_abs(proximity_dev, ABS_DISTANCE, 0);
			input_sync(proximity_dev);
		}
#endif
//			wake_unlock(&gsl_wakelock);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x5a;
			ps_store_data[2] = 0x5a;
			ps_store_data[1] = 0x5a;
			ps_store_data[0] = 0x5a;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);

			printk("[GSL] The value of {0x03,0x00} is = %x %x %x %x\n",ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);	

			/* Writing 0x02 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x02;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);
			device_init_wakeup(&ts_data->client->dev, 1);
			printk("[GSL] The value of {0x04,0x00} is = %x %x %x %x\n",ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);
	}
	else 
	{
//			wake_unlock(&gsl_wakelock);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[3];
			ps_store_data[2] = gsl_psensor_data[2];
			ps_store_data[1] = gsl_psensor_data[1];
			ps_store_data[0] = gsl_psensor_data[0];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);

			/* Writing 0x00 put into {0xf0,0x04} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[7];
			ps_store_data[2] = gsl_psensor_data[6];
			ps_store_data[1] = gsl_psensor_data[5];
			ps_store_data[0] = gsl_psensor_data[4];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);
			device_init_wakeup(&ts_data->client->dev, 0);
	}
	return 0;
}

ssize_t gsl_proximity_enable_show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
	return sprintf(pBuf, "%d",gsl_enabled);
}

ssize_t gsl_proximity_enable_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	int enable;
	u8 ps_store_data[4]={0};
	if (pBuf != NULL)
	{
		printk("gsl::Enter %s\n",__func__);
		sscanf(pBuf, "%d", &enable);
		//gsl_prox_sensor_enable(ts_data->client, enable);
		//vps_set_enable(enable);
			/* Writing 0x02 put into {0xf0,0x03} */
		if(enable == 1)
		{

//			wake_unlock(&gsl_wakelock);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x5a;
			ps_store_data[2] = 0x5a;
			ps_store_data[1] = 0x5a;
			ps_store_data[0] = 0x5a;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);

			printk("[GSL] The value of {0x03,0x00} is = %x %x %x %x\n",ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);	

			/* Writing 0x02 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x02;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);

			printk("[GSL] The value of {0x04,0x00} is = %x %x %x %x\n",ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);
	}
		else
		{
//			wake_unlock(&gsl_wakelock);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[3];
			ps_store_data[2] = gsl_psensor_data[2];
			ps_store_data[1] = gsl_psensor_data[1];
			ps_store_data[0] = gsl_psensor_data[0];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);

			/* Writing 0x00 put into {0xf0,0x04} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[7];
			ps_store_data[2] = gsl_psensor_data[6];
			ps_store_data[1] = gsl_psensor_data[5];
			ps_store_data[0] = gsl_psensor_data[4];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);	
	}
	}
	return nSize;
}

static DEVICE_ATTR(enable, 0666, gsl_proximity_enable_show, gsl_proximity_enable_store);

#if 0
ssize_t proximity_vendor_show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{

	return 0;//sprintf(pBuf, "%s", vps->vendor);
}

ssize_t proximity_vendor_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	extern struct input_dev* get_ido3_taos_input_device(void);
	//extern struct input_dev* get_stk_input_device(void);
	int value;
	printk("proximity_vendor_store in!\n");
	if (pBuf != NULL)
	{
		sscanf(pBuf, "%d", &value);
	//	if(vps->value == 0 || vps->value >= TOTAL)
	//		printk("wrong vendor value!\n");

		//switch(vps->value)
		if(value)
		{
	//		case TAOS:
				proximity_dev = get_ido3_taos_input_device();
				if(vps->proximity_dev == NULL)
					printk("proximity input device is NULL!\n");
				printk("gsl priximity sensor success\n");
				break;
			//case STK:
			//	vps->proximity_dev = get_stk_input_device();
			//	if(vps->proximity_dev == NULL)
			//		printk("proximity input device is NULL!\n");
			//	printk("STK priximity sensor\n");
			//	break;

			default:
				printk("error proximity vendor!\n");
				break;
		}
	}
	return nSize;
}

static DEVICE_ATTR(vendor, 0666, proximity_vendor_show, proximity_vendor_store);


ssize_t gsl_proximity_function_enable_show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
	return sprintf(pBuf, "%x", vps->proximity_function);
}

ssize_t gsl_proximity_function_enable_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	u32 nProximityMode;
	if (pBuf != NULL)
	{
		sscanf(pBuf, "%x", &nProximityMode);
		printk("nWakeupMode = 0x%x\n", nProximityMode);
		vps->proximity_function = nProximityMode;
		//tp_prox_sensor_enable(g_ft5x06_ts_data->client, nProximityMode)
		//DrvMainFirmwareProximityEnable(nProximityMode);
	}
	return nSize;
}

static DEVICE_ATTR(proximity, 0666, gsl_proximity_function_enable_show, gsl_proximity_function_enable_store);
#endif
#if 1
static int sys_device_create(void)
{
	struct class *virtual_proximity = NULL;
	struct device *virtual_proximity_device = NULL;

	virtual_proximity = class_create(THIS_MODULE, "virtual-proximity");
	if (IS_ERR(virtual_proximity))
		printk("Failed to create class(virtual_proximity)!\n");

	virtual_proximity_device = device_create(virtual_proximity, NULL, 0, NULL, "device");
	if (IS_ERR(virtual_proximity_device))
		printk("Failed to create device(virtual_proximity_device)!\n");

	if (device_create_file(virtual_proximity_device, &dev_attr_enable) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

//	if (device_create_file(virtual_proximity_device, &dev_attr_vendor) < 0)
//		printk("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

//	if (device_create_file(virtual_proximity_device, &dev_attr_proximity) < 0)
//		printk("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

	return 0;
}
#endif
#endif

#ifdef GSL_TEST_TP
void gsl_I2C_ROnePage(unsigned int addr, char *buf)
{
	u8 tmp_buf[4]={0};
	tmp_buf[3]=(u8)(addr>>24);
	tmp_buf[2]=(u8)(addr>>16);
	tmp_buf[1]=(u8)(addr>>8);
	tmp_buf[0]=(u8)(addr);
	gsl_write_interface(ts_data->client,0xf0,tmp_buf,4);
	gsl_ts_read(ts_data->client,0,buf,128);
}
EXPORT_SYMBOL(gsl_I2C_ROnePage);
void gsl_I2C_RTotal_Address(unsigned int addr,unsigned int *data)
{
	u8 tmp_buf[4]={0};	
	tmp_buf[3]=(u8)((addr/0x80)>>24);
	tmp_buf[2]=(u8)((addr/0x80)>>16);
	tmp_buf[1]=(u8)((addr/0x80)>>8);
	tmp_buf[0]=(u8)((addr/0x80));
	gsl_write_interface(ts_data->client,0xf0,tmp_buf,4);
	gsl_ts_read(ts_data->client,addr%0x80,tmp_buf,4);
	*data = tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
}
EXPORT_SYMBOL(gsl_I2C_RTotal_Address);
#endif

#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	unsigned int tmp=0;
	
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<gsl_cfg_table[gsl_cfg_index].data_size)
				seq_printf(m,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]); 
#endif
		}
		else 
		{
			printk("GSL***%s*****1 = %02x, 2 = %02x, 3= %02x , 4 = %02x,***\n",__func__,
				gsl_data_proc[0],gsl_data_proc[1],gsl_data_proc[2],gsl_data_proc[3]);

			tmp = (gsl_data_proc[7]<<24) + (gsl_data_proc[6]<<16) + (gsl_data_proc[5]<<8) + gsl_data_proc[4];
			if(tmp >= 1 && gsl_data_proc[0] == 0x7c)
			{
				tmp -= 1;
				gsl_data_proc[7] = (tmp>>24) & 0xff;
				gsl_data_proc[6] = (tmp>>16) & 0xff;
				gsl_data_proc[5] = (tmp>> 8) & 0xff;
				gsl_data_proc[4] = (tmp>> 0) & 0xff;
			}
			gsl_write_interface(ts_data->client,0Xf0,&gsl_data_proc[4],4);

			if(gsl_data_proc[0] < 0x80)
				gsl_ts_read(ts_data->client,gsl_data_proc[0],temp_data,4);

			gsl_ts_read(ts_data->client,gsl_data_proc[0],temp_data,4);

			printk("GSL***%s*****1 = %02x, 2 = %02x, 3= %02x , 4 = %02x,***\n",__func__,temp_data[0],temp_data[1],temp_data[2],temp_data[3]);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
//	*eof = 1;
	return (0);
}
static int gsl_config_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	dev_info("[tp-gsl][%s] \n",__func__);


	if(count > 512)
	{
		dev_info("size not match [%d:%d]\n", CONFIG_LEN, count);
        return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}
	if(copy_from_user(path_buf, buffer, count))
	{
		dev_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	dev_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	printk("[tp-GSL][%s][%s]\n",__func__,temp_buf);	

	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);

	printk("GSL*****%s**buf[0] = %d, buf[1] = %d ,buf[2] = %d, buf[3] = %d ,buf[4] = %d, buf[5] = %d ,buf[6] = %d, buf[7] = %d **\n",__func__,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);	
		
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
	#ifdef GSL_TIMER	
		cancel_delayed_work_sync(&gsl_timer_check_work);
	#endif
		gsl_proc_flag = 1;
		gsl_reset_core(ts_data->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		gsl_reset_core(ts_data->client);
		gsl_start_core(ts_data->client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_write_interface(ts_data->client,buf[4],buf,4);
	}
#ifdef GSL_ALG_ID
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<gsl_cfg_table[gsl_cfg_index].data_size)
		{
			gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif


#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{
	struct gsl_ts_data *ts = ts_data;
	struct i2c_client *gsl_client = ts->client;
	static int i2c_lock_flag = 0;
	char read_buf[4]  = {0};
	char init_chip_flag = 0;
	int i,flag;
	dev_info("----------------gsl_monitor_worker-----------------\n");	

	if(i2c_lock_flag != 0)
		return;
	else
		i2c_lock_flag = 1;

	/*check 0xb4 register,check interrupt if ok*/
	gsl_ts_read(gsl_client, 0xb4, read_buf, 4);
	memcpy(int_2nd,int_1st,4);
	memcpy(int_1st,read_buf,4);

	if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&
		int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0])
	{
		printk("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",
			int_1st[3], int_1st[2], int_1st[1], int_1st[0], 
			int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
	
	/*check 0xb0 register,check firmware if ok*/
	for(i=0;i<5;i++){
		gsl_ts_read(gsl_client, 0xb0, read_buf, 4);

		printk("[%s]:0xb0 before judgment = {0x%02x%02x%02x%02x} \n",
				__func__,read_buf[3],read_buf[2],read_buf[1],read_buf[0]);
		
		if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || 
			read_buf[1] != 0x5a || read_buf[0] != 0x5a){
			
			printk("[%s]:0xb0 after judgment = {0x%02x%02x%02x%02x} \n",
				__func__,read_buf[3],read_buf[2],read_buf[1],read_buf[0]);
			
			flag = 1;
		}else{
			flag = 0;
			break;
		}

	}
	if(flag == 1){
		init_chip_flag = 1;
		goto queue_monitor_work;
	}
	
	/*check 0xbc register,check dac if normal*/
	
	for(i=0;i<5;i++){
		gsl_ts_read(gsl_client, 0xbc, read_buf, 4);
		
		printk("[%s]:0xbc before judgment = {0x%02x%02x%02x%02x} \n",
			__func__,read_buf[3],read_buf[2],read_buf[1],read_buf[0]);

		if(read_buf[3] != 0 || read_buf[2] != 0 || 
			read_buf[1] != 0 || read_buf[0] != 0){
			
			printk("[%s]:0xbc after judgment = {0x%02x%02x%02x%02x} \n",
				__func__,read_buf[3],read_buf[2],read_buf[1],read_buf[0]);

			flag = 1;
		}else{
			flag = 0;
			break;
		}
	}
	if(flag == 1){
		gsl_reset_core(gsl_client);
		gsl_start_core(gsl_client);
		init_chip_flag = 0;
	}
queue_monitor_work:
	if(init_chip_flag){
		gsl_sw_init(gsl_client);
		memset(int_1st,0xff,sizeof(int_1st));
	}
	
	if(!ts_data->suspended){
		queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 200);
	}
	i2c_lock_flag = 0;

}
#endif

static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4];
	int i,err;
	for(i = 0; i < 5; i++) {
		err = gsl_ts_read(client, 0xfc, buf, 4);
		printk("[tp-gsl] 0xfc = {0x%02x%02x%02x%02x}\n", buf[3], buf[2],
			buf[1],buf[0]);
		if(!(err < 0)) {
			err = 1;
			break;	
		}
	}
	return err;	
}

#ifdef GSL_COMPATIBLE_GPIO
static int gsl_read_TotalAdr(struct i2c_client *client,u32 addr,u32 *data)
{
	u8 buf[4];
	int err;
	buf[3]=(u8)((addr/0x80)>>24);
	buf[2]=(u8)((addr/0x80)>>16);
	buf[1]=(u8)((addr/0x80)>>8);
	buf[0]=(u8)((addr/0x80));
	gsl_write_interface(client,0xf0,buf,4);
	err = gsl_ts_read(client,addr%0x80,buf,4);
	if(err > 0){
		*data = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
	}
	return err;
}
static int gsl_write_TotalAdr(struct i2c_client *client,u32 addr,u32 *data)
{
	int err;
	u8 buf[4];
	u32 value = *data;
	buf[3]=(u8)((addr/0x80)>>24);
	buf[2]=(u8)((addr/0x80)>>16);
	buf[1]=(u8)((addr/0x80)>>8);
	buf[0]=(u8)((addr/0x80));
	gsl_write_interface(client,0xf0,buf,4);
	buf[3]=(u8)((value)>>24);
	buf[2]=(u8)((value)>>16);
	buf[1]=(u8)((value)>>8);
	buf[0]=(u8)((value));
	err = gsl_write_interface(client,addr%0x80,buf,4);
	return err;
}

static int gsl_gpio_idt_tp(struct i2c_client *client)
{
	int i;
	u32 value = 0;
	u8 rstate;
	value = 0x1;
	gsl_write_TotalAdr(client,0xff000018,&value);
	value = 0x0;
	gsl_write_TotalAdr(client,0xff020000,&value);
	for(i=0;i<3;i++){
		gsl_read_TotalAdr(client,0xff020004,&value);
	}
	rstate = value & 0x1;
	
	if(rstate == 1){
		 gsl_cfg_index  = 0;   //\B5۾\A7
	}
	else if(rstate == 0){
		gsl_cfg_index = 1;   //\B4\B4\CA\C0
	}

	dev_info("[tpd-gsl][%s] [rstate]=[%d]\n",__func__,rstate);
	return 1;
}
#endif

#ifdef GSL_TEST_TP
static ssize_t gsl_test_show(void)
{
	static int gsl_test_flag = 0; 
	char *tmp_buf;
	int err;
	int result = 0;
	printk("[%s]:enter gsl_test_show start::gsl_test_flag  = %d\n",__func__,gsl_test_flag);
	if(gsl_test_flag == 1){
		return 0;	
	}
	gsl_test_flag = 1;
	tmp_buf = kzalloc(3*1024,GFP_KERNEL);
	if(!tmp_buf){
		printk("[%s]:kzalloc kernel fail\n",__func__);
		return 0;
		}
	
	printk("[%s]:tp module test begin\n",__func__);
	
	err = gsl_tp_module_test(tmp_buf,3*1024);

	printk("[%s]:enter gsl_test_show end\n",__func__);
	
	if(err > 0){
		printk("[%s]:tp test pass\n",__func__);
		result = 1;

	}else{
		printk("[%s]:tp test failure\n",__func__);
		result = 0;
	}
	kfree(tmp_buf);
	gsl_test_flag = 0; 
	return result;
}

static s32 gsl_openshort_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}

static s32 gsl_openshort_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	char *ptr = buf;
	int test_result  = 0;
	if(*ppos)
	{
		printk("[%s]:tp test again return\n",__func__);
		return 0;
	}
	*ppos += count;
	test_result = gsl_test_show();
	memset(buf,'\0',16);
	if(1 == test_result)
	{
		printk("[%s]:tp test pass\n",__func__);
//		sprintf(ptr, "result=%d\n", 1);
	}
	else
	{
		printk("[%s]:tp test failure\n",__func__);
//		sprintf(ptr, "result=%d\n", 0);
	}
//	return count;
	return sprintf(ptr, "result=%d\n", test_result);
}

static ssize_t gsl_rawdata_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}

static ssize_t gsl_rawdata_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
//	int i,number=0;
	int i;
	static short* gsl_ogv;
	ssize_t read_buf_chars = 0; 
	int ret  = 0;
	gsl_ogv = kzalloc(26*14*2,GFP_KERNEL);
	if(!gsl_ogv){
		return -1;
	}  

	//printk("[%s]:rawdata proc node read!\n",__func__);

#if 0
	if( number != 0 )
		return -1;
	else
		number++;
#endif	
#if 1
	if(*ppos)
	{
		printk("[%s]:tp test again return\n",__func__);
		return 0;
	}
#endif
	//wu
	ret=gsl_test_show();
	gsl_obtain_array_data_ogv(gsl_ogv,26,14);

	for(i=0;i<26*14;i++)
	{
		read_buf_chars += sprintf(&(buf[read_buf_chars])," _%u_ ",gsl_ogv[i]);
		if(!((i+1)%10))
		{
			buf[read_buf_chars++] = '\n';
		}

	}

	buf[read_buf_chars-1] = '\n';

	//printk("[%s]:rawdata proc node end!\n",__func__);
	
	//*ppos += count;
	//return count;
	*ppos += read_buf_chars; 


	return read_buf_chars; 
}

static const struct file_operations gsl_rawdata_procs_fops =
{
	.write = gsl_rawdata_proc_write,
	.read = gsl_rawdata_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations gsl_openshort_procs_fops =
{
    .write = gsl_openshort_proc_write,
    .read = gsl_openshort_proc_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

void create_ctp_proc(void)
{
	struct proc_dir_entry *gsl_device_proc = NULL;
	struct proc_dir_entry *gsl_openshort_proc = NULL;
	struct proc_dir_entry *gsl_rawdata_proc = NULL;

	gsl_device_proc = proc_mkdir(GSL_PARENT_PROC_NAME, NULL);
		if(gsl_device_proc == NULL)
    	{
        	printk("[%s]: create parent_proc fail\n",__func__);
        	return;
    	}

	gsl_openshort_proc = proc_create(GSL_OPENHSORT_PROC_NAME, 0666, gsl_device_proc, &gsl_openshort_procs_fops);

    	if (gsl_openshort_proc == NULL)
    	{
        	printk("[%s]: create openshort_proc fail\n",__func__);
    	}
		
	gsl_rawdata_proc = proc_create(GSL_RAWDATA_PROC_NAME, 0777, gsl_device_proc, &gsl_rawdata_procs_fops);
    	if (gsl_rawdata_proc == NULL)
    	{
        	printk("[%s]: create ctp_rawdata_proc fail\n",__func__);
    	}
}

#endif

static int gsl_ts_power_init(struct gsl_ts_data *ts_data, bool init)
{
	int rc;

	if (init) {
		ts_data->vdd = regulator_get(&ts_data->client->dev,
									"vdd");
		if (IS_ERR(ts_data->vdd)) {
			rc = PTR_ERR(ts_data->vdd);
			dev_err(&ts_data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(ts_data->vdd) > 0) {
			rc = regulator_set_voltage(ts_data->vdd,
							GSL_VTG_MIN_UV,
							GSL_VTG_MAX_UV);
			if (rc) {
				dev_err(&ts_data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		ts_data->vcc_i2c = regulator_get(&ts_data->client->dev,
								"vcc-i2c");
		if (IS_ERR(ts_data->vcc_i2c)) {
			rc = PTR_ERR(ts_data->vcc_i2c);
			dev_err(&ts_data->client->dev,
				"Regulator get failed vcc-i2c rc=%d\n", rc);
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(ts_data->vcc_i2c,
						GSL_I2C_VTG_MIN_UV,
						GSL_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&ts_data->client->dev,
				"Regulator set_vtg failed vcc-i2c rc=%d\n", rc);
				goto reg_vcc_i2c_put;

			}
		}
	} else {
		if (regulator_count_voltages(ts_data->vdd) > 0)
			regulator_set_voltage(ts_data->vdd, 0,
							GSL_VTG_MAX_UV);

		regulator_put(ts_data->vdd);

		if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
			regulator_set_voltage(ts_data->vcc_i2c, 0,
						GSL_I2C_VTG_MAX_UV);

		regulator_put(ts_data->vcc_i2c);
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(ts_data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(ts_data->vdd) > 0)
		regulator_set_voltage(ts_data->vdd, 0, GSL_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(ts_data->vdd);
	return rc;
}

static int gsl_ts_power_on(struct gsl_ts_data *ts_data, bool on)
{

	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(ts_data->vdd);
	if (rc) {
		dev_err(&ts_data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(ts_data->vcc_i2c);
	if (rc) {
		dev_err(&ts_data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(ts_data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(ts_data->vdd);
	if (rc) {
		dev_err(&ts_data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(ts_data->vcc_i2c);
	if (rc) {
		dev_err(&ts_data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(ts_data->vdd);
	}

	return rc;
}

static void gsl_sw_init(struct i2c_client *client)
{
	if(1==ts_data->updating_fw)
		return;
	ts_data->updating_fw = 1;

	gpio_set_value(ts_data->pdata->reset_gpio, 0);
	msleep(20);
	gpio_set_value(ts_data->pdata->reset_gpio, 1);
	msleep(20);

	gsl_clear_reg(client);
	gsl_reset_core(client);
	{
	gsl_load_fw(client,gsl_cfg_table[gsl_cfg_index].fw,
		gsl_cfg_table[gsl_cfg_index].fw_size);
	}
	gsl_start_core(client);

	ts_data->updating_fw = 0;
}

static void check_mem_data(struct i2c_client *client)
{

	u8 read_buf[4]  = {0};
	msleep(30);
	gsl_ts_read(client,0xb0,read_buf,4);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a 
		|| read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk("0xb4 ={0x%02x%02x%02x%02x}\n",
			read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		gsl_sw_init(client);
	}
}

#define GSL_CHIP_NAME	"gslx68x"
static ssize_t gsl_sysfs_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	//ssize_t len=0;
	int count = 0;
	u8 buf_tmp[4];
	u32 tmp;
	//char *ptr = buf;
	count += scnprintf(buf,PAGE_SIZE,"sileadinc:");
	count += scnprintf(buf+count,PAGE_SIZE-count,GSL_CHIP_NAME);

#ifdef GSL_TIMER
	count += scnprintf(buf+count,PAGE_SIZE-count,":0001-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,":0001-0:");
#endif

#ifdef TPD_PROC_DEBUG
	count += scnprintf(buf+count,PAGE_SIZE-count,"0002-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,"0002-0:");
#endif

#ifdef GSL_PROXIMITY_SENSOR//TPD_PROXIMITY
	count += scnprintf(buf+count,PAGE_SIZE-count,"0003-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,"0003-0:");
#endif

#ifdef GSL_DEBUG
	count += scnprintf(buf+count,PAGE_SIZE-count,"0004-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,"0004-0:");
#endif

#ifdef GSL_ALG_ID
	tmp = gsl_version_id();
	count += scnprintf(buf+count,PAGE_SIZE-count,"%08x:",tmp);
	count += scnprintf(buf+count,PAGE_SIZE-count,"%08x:",
		gsl_cfg_table[gsl_cfg_index].data_id[0]);
#endif
	buf_tmp[0]=0x3;buf_tmp[1]=0;buf_tmp[2]=0;buf_tmp[3]=0;
	gsl_write_interface(ts_data->client,0xf0,buf_tmp,4);
	gsl_ts_read(ts_data->client,0,buf_tmp,4);
	count += scnprintf(buf+count,PAGE_SIZE-count,"%02x%02x%02x%02x\n",
		buf_tmp[3],buf_tmp[2],buf_tmp[1],buf_tmp[0]);

    return count;
}

static DEVICE_ATTR(version, 0444, gsl_sysfs_version_show, NULL);

#ifdef GSL_PROXIMITY_SENSOR
char flag_tp_down = 0;
static struct wake_lock ps_wake_lock;
char ps_data_state[1] = {1};
static int proximity_enable = 0;
static u8 gsl_psensor_data[8]={0};
enum
{
       DISABLE_CTP_PS = 0,
       ENABLE_CTP_PS = 1,
       RESET_CTP_PS
};

static void gsl_gain_psensor_data(struct i2c_client *client)
{
	int tmp = 0;
	u8 buf[4]={0};

	buf[0]=0x3;
	gsl_write_interface(client,0xf0,buf,4);
	tmp = gsl_write_interface(client,0x0,&gsl_psensor_data[0],4);
	if(tmp <= 0){
		 gsl_write_interface(client,0x0,&gsl_psensor_data[0],4);
	}

	buf[0]=0x4;
	gsl_write_interface(client,0xf0,buf,4);
	tmp = gsl_write_interface(client,0x0,&gsl_psensor_data[4],4);
	if(tmp <= 0){
		gsl_write_interface(client,0x0,&gsl_psensor_data[4],4);
	}
	
}

static ssize_t show_proximity_sensor_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s : GSL_proximity_sensor_status = %d\n",__func__,ps_data_state[0]);
	return sprintf(buf, "0x%02x   \n",ps_data_state[0]);
}

static ssize_t show_proximity_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s : GSL_proximity_enable = %d\n",__func__,proximity_enable);
	return sprintf(buf, "0x%02x   \n",proximity_enable);
}
static ssize_t store_proximity_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int rc;
	int val;
	u8 ps_store_data[4];
	if(buf != NULL && size != 0)
	{
		sscanf(buf,"%d",&val);
		printk("%s : val =%d\n",__func__,val);
		if(DISABLE_CTP_PS == val)
		{
			proximity_enable = 0;
			printk("DISABLE_CTP_PS buf=%d,size=%d,val=%d\n", *buf, size,val);

			/* Writing 0x00 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[3];
			ps_store_data[2] = gsl_psensor_data[2];
			ps_store_data[1] = gsl_psensor_data[1];
			ps_store_data[0] = gsl_psensor_data[0];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);

			/* Writing 0x00 put into {0xf0,0x04} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[7];
			ps_store_data[2] = gsl_psensor_data[6];
			ps_store_data[1] = gsl_psensor_data[5];
			ps_store_data[0] = gsl_psensor_data[4];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);
			
			if (rc < 0)
			{
				printk("%s :write err val =0 ++++++++++++++++\n",__func__);
			}
			msleep(200);
			printk("RESET_CTP_PS buf=%d\n", *buf);
			wake_lock(&ps_wake_lock);
		}

		else if(ENABLE_CTP_PS == val)
		{
			wake_lock(&ps_wake_lock);
			proximity_enable = 1;
			printk("ENABLE_CTP_PS buf=%d,size=%d,val=%d\n", *buf, size,val);

			/* Writing 0x5a5a5a5a put into {0xf0,0x03}*/
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x5a;
			ps_store_data[2] = 0x5a;
			ps_store_data[1] = 0x5a;
			ps_store_data[0] = 0x5a;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);

			printk("[GSL] The value of {0x03,0x00} is = %x %x %x %x\n",buf[3],buf[2],buf[1],buf[0]);	

			/* Writing 0x02 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x02;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);

			printk("[GSL] The value of {0x04,0x00} is = %x %x %x %x\n",buf[3],buf[2],buf[1],buf[0]);
			
			if (rc < 0)
			{
				printk("%s :write err++++++++++++++++\n",__func__);
			}
		}
	}

	return size;
}

static int psensor_ps_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	int rc;
	u8 ps_store_data[4]={0};
	int i=0;

		if(DISABLE_CTP_PS == enable)
		{
			i=0;
			proximity_enable = 0;
			ps_data_state[0] = 1;

			/* Writing 0x00 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[3];
			ps_store_data[2] = gsl_psensor_data[2];
			ps_store_data[1] = gsl_psensor_data[1];
			ps_store_data[0] = gsl_psensor_data[0];
			gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);

			/* Writing 0x00 put into {0xf0,0x04} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			rc = gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = gsl_psensor_data[7];
			ps_store_data[2] = gsl_psensor_data[6];
			ps_store_data[1] = gsl_psensor_data[5];
			ps_store_data[0] = gsl_psensor_data[4];
			rc = gsl_write_interface(ts_data->client,0,&ps_store_data[0],4);
			
			if (rc < 0)
			{
				printk("%s :write err val =0 ++++++++++++++++\n",__func__);
			}
			msleep(200);
			wake_unlock(&ps_wake_lock);
		}

		else if(ENABLE_CTP_PS == enable)
		{
			i=0;
			wake_lock(&ps_wake_lock);
			proximity_enable = 1;

			ps_data_state[0] = 1;

			/* Writing 0x5a5a5a5a put into {0xf0,0x03}*/
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x5a;
			ps_store_data[2] = 0x5a;
			ps_store_data[1] = 0x5a;
			ps_store_data[0] = 0x5a;
			gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);

			printk("[GSL] The value of {0x03,0x00} is = %x %x %x %x\n",ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);	

			/* Writing 0x02 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			rc = gsl_write_interface(ts_data->client,0xf0,&ps_store_data[0],4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x02;
			rc = gsl_write_interface(ts_data->client,0x00,&ps_store_data[0],4);
			
			if (rc < 0)
			{
				printk("%s :write err++++++++++++++++\n",__func__);
			}
		}

	return 0;
}

static DEVICE_ATTR(proximity_sensor_enable, 0777, show_proximity_sensor_enable, store_proximity_sensor_enable);
static DEVICE_ATTR(proximity_sensor_status, 0777, show_proximity_sensor_status,NULL);

#endif


#ifdef GSL_GESTURE
static void gsl_enter_doze(struct gsl_ts_data *ts)
{
	u8 buf[4] = {0};
#if 0
	u32 tmp;
	gsl_reset_core(ts->client);
	temp = ARRAY_SIZE(GSLX68X_FW_GESTURE);
	gsl_load_fw(ts->client,GSLX68X_FW_GESTURE,temp);
	gsl_start_core(ts->client);
	msleep(1000);		
#endif

	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_write_interface(ts->client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0x1;
	buf[3] = 0x5a;
	gsl_write_interface(ts->client,0x8,buf,4);
	//gsl_gesture_status = GE_NOWORK;
	msleep(10);
	gsl_gesture_status = GE_ENABLE;

}
static void gsl_quit_doze(struct gsl_ts_data *ts)
{
	u8 buf[4] = {0};
	//u32 tmp;

	gsl_gesture_status = GE_DISABLE;
		
	gpio_set_value(GSL_RST_GPIO_NUM,0);
	msleep(20);
	gpio_set_value(GSL_RST_GPIO_NUM,1);
	msleep(5);
	
	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_write_interface(ts->client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0x5a;
	gsl_write_interface(ts->client,0x8,buf,4);
	msleep(10);

#if 0
	gsl_reset_core(ts_data->client);
	temp = ARRAY_SIZE(GSLX68X_FW_CONFIG);
	//gsl_load_fw();
	gsl_load_fw(ts_data->client,GSLX68X_FW_CONFIG,temp);
	gsl_start_core(ts_data->client);
#endif
}

static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 count = 0;
	count += scnprintf(buf,PAGE_SIZE,"tp gesture is on/off:\n");
	if(gsl_gesture_flag == 1){
		count += scnprintf(buf+count,PAGE_SIZE-count,
				" on \n");
	}else if(gsl_gesture_flag == 0){
		count += scnprintf(buf+count,PAGE_SIZE-count,
				" off \n");
	}
	count += scnprintf(buf+count,PAGE_SIZE-count,"tp gesture:");
	count += scnprintf(buf+count,PAGE_SIZE-count,
			"%c\n",gsl_gesture_c);
    	return count;
}
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#if 1
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;  
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
	}
#endif
	return count;
}
static DEVICE_ATTR(gesture, 0666, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);
#endif

static struct attribute *gsl_attrs[] = {

	&dev_attr_version.attr,
		
#ifdef GSL_GESTURE
	&dev_attr_gesture.attr,
#endif

#ifdef GSL_PROXIMITY_SENSOR
	&dev_attr_proximity_sensor_enable.attr,
	&dev_attr_proximity_sensor_status.attr,
#endif

	NULL
};
static const struct attribute_group gsl_attr_group = {
	.attrs = gsl_attrs,
};

#if GSL_HAVE_TOUCH_KEY
static int gsl_report_key(struct input_dev *idev,int x,int y)
{
	int i;
	for(i=0;i<GSL_KEY_NUM;i++)
	{
		if(x > gsl_key_data[i].x_min &&
			x < gsl_key_data[i].x_max &&
			y > gsl_key_data[i].y_min &&
			y < gsl_key_data[i].y_max)
		{
			ts_data->gsl_key_state = i+1;
			input_report_key(idev,gsl_key_data[i].key,1);
			input_sync(idev);
			return 1;
		}
	}
	return 0;
}
#endif
static void gsl_report_point(struct input_dev *idev, struct gsl_touch_info *cinfo)
{
	int i; 
	u32 gsl_point_state = 0;
	u32 temp=0;
	if(cinfo->finger_num>0 && cinfo->finger_num<6)
	{
		ts_data->gsl_up_flag = 0;
		gsl_point_state = 0;
	#if GSL_HAVE_TOUCH_KEY
		if(1==cinfo->finger_num)
		{
			if(cinfo->x[0] > GSL_MAX_X || cinfo->y[0] > GSL_MAX_Y)
			{
				gsl_report_key(idev,cinfo->x[0],cinfo->y[0]);
				return;		
			}
		}
	#endif
		for(i=0;i<cinfo->finger_num;i++)
		{
			gsl_point_state |= (0x1<<cinfo->id[i]);	
			//dev_info("id = %d, x = %d, y = %d \n",cinfo->id[i], 
				//cinfo->x[i],cinfo->y[i]);
		#ifdef GSL_REPORT_POINT_SLOT
			input_mt_slot(idev, cinfo->id[i] - 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
		
		#else 
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(idev);		
		#endif
		}
	}
	else if(cinfo->finger_num == 0)
	{
		gsl_point_state = 0;
	//	ts_data->gsl_point_state = 0;
		if(1 == ts_data->gsl_up_flag)
		{
			return;
		}
		ts_data->gsl_up_flag = 1;

	#ifdef GSL_PROXIMITY_SENSOR
		input_report_abs(idev, ABS_DISTANCE, ps_data_state[0]);
		input_sync(idev);
	#endif
		
	#if GSL_HAVE_TOUCH_KEY
		if(ts_data->gsl_key_state > 0)
		{
			if(ts_data->gsl_key_state < GSL_KEY_NUM+1)
			{
				input_report_key(idev,gsl_key_data[ts_data->gsl_key_state - 1].key,0);
				input_sync(idev);
			}
		}
	#endif
	#ifndef GSL_REPORT_POINT_SLOT
		input_report_key(idev, BTN_TOUCH, 0);
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(idev);
	#endif
	}

	temp = gsl_point_state & ts_data->gsl_point_state;
	temp = (~temp) & ts_data->gsl_point_state;

	#ifdef GSL_REPORT_POINT_SLOT
	for(i=1;i<6;i++)
	{
		if(temp & (0x1<<i))
		{
			input_mt_slot(idev, i-1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, false);
		}
	}
	#endif	
	
	ts_data->gsl_point_state = gsl_point_state;
	input_sync(idev);
}

static void gsl_report_work(struct work_struct *work)
{
	int rc,tmp;
	u8 buf[44] = {0};
	struct gsl_touch_info *cinfo = ts_data->cinfo;
	struct i2c_client *client = ts_data->client;
	struct input_dev *idev = ts_data->input_dev;
	int tmp1=0;
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	u8 proximity_buf[4] = {0};
#endif	
	#ifdef GSL_GESTURE
	unsigned int test_count = 0;
	#endif

	#ifdef GSL_PROXIMITY_SENSOR
	u8 tmp_prox = 0;
	#endif
	
	if(1 == ts_data->updating_fw)
		goto schedule;
	
	#ifdef TPD_PROC_DEBUG
		if(gsl_proc_flag == 1){
			return;
		}
	#endif
	
	/* Proximity Sensor */
	#ifdef GSL_PROXIMITY_SENSOR
	tmp_prox = gsl_ts_read(client, 0xac, buf, 4);
	if(proximity_enable==1)
	{
          if(tmp_prox == 0x01) 		/* Closing */
          {
            ps_data_state[0] = 0;
			input_report_abs(ts_data->input_dev_ps, ABS_DISTANCE, ps_data_state[0]);
			input_sync(ts_data->input_dev_ps);
			return;
          }
          else if(tmp_prox == 0x00) /* Leaving*/
          {
            ps_data_state[0] = 1;
			input_report_abs(ts_data->input_dev_ps, ABS_DISTANCE, ps_data_state[0]);
			input_sync(ts_data->input_dev_ps);
			return;
          }
	}
	#endif
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
//	printk("Enter %s,gsl_enabled = %d\n",__func__,gsl_enabled);
	if(gsl_enabled)
	{
		gsl_ts_read(ts_data->client, 0xac,proximity_buf,4);
		//printk("proxi_fts 0xB0 state value is0x%02X\n", reg_value);
		if(proximity_buf[0] == 1)
	//	{
	//		tp_prox_sensor_enable(data->client, 1);
	//	}
	//	ft5x0x_read_reg(data->client, 0x01, &proximity_status);
	//	if(proximity_status == 0xC0)
		{
			printk("!!!*** it is near\n");
			input_report_abs(proximity_dev, ABS_DISTANCE, 0);
			input_sync(proximity_dev);
		}
		else //if(proximity_buf[0] == 0)
		{
			printk("!!!^^^ it is far \n");
			wake_lock_timeout(&GSL_wakelock, 1*HZ);
			input_report_abs(proximity_dev, ABS_DISTANCE, 1);
			input_sync(proximity_dev);
		}
	}
#endif

	/* Gesture Resume */
	#ifdef GSL_GESTURE
		printk("GSL:::0x80=%02x%02x%02x%02x[%d]\n",buf[3],buf[2],buf[1],buf[0],test_count++);
		printk("GSL:::0x84=%02x%02x%02x%02x\n",buf[7],buf[6],buf[5],buf[4]);
		printk("GSL:::0x88=%02x%02x%02x%02x\n",buf[11],buf[10],buf[9],buf[8]);
	#endif
	
	/* read data from DATA_REG */
	rc = gsl_ts_read(client, 0x80, buf, 44);
	if (rc < 0) 
	{
		dev_err(&client->dev, "[gsl] I2C read failed\n");
		goto schedule;
	}

	if (buf[0] == 0xff) {
		goto schedule;
	}

	cinfo->finger_num = buf[0];
	for(tmp=0;tmp<(cinfo->finger_num>10 ? 10:cinfo->finger_num);tmp++)
	{
		cinfo->y[tmp] = (buf[tmp*4+4] | ((buf[tmp*4+5])<<8));
		cinfo->x[tmp] = (buf[tmp*4+6] | ((buf[tmp*4+7] & 0x0f)<<8));
		cinfo->id[tmp] = buf[tmp*4+7] >> 4;
		//dev_info("tp-gsl  x = %d y = %d \n",cinfo->x[tmp],cinfo->y[tmp]);
	}
	
	//dev_info("111 finger_num= %d\n",cinfo->finger_num);
#ifdef GSL_ALG_ID
	cinfo->finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	gsl_alg_id_main(cinfo);
	tmp1=gsl_mask_tiaoping();
	//dev_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		buf[1]=0;
		buf[2]=0;
		buf[3]=0;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		//dev_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n", tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_write_interface(client,0x8,buf,4);
	}
#endif

#ifdef GSL_GESTURE
		printk("gsl_gesture_status=%d,gsl_gesture_flag=%d[%d]\n",gsl_gesture_status,gsl_gesture_flag,test_count++);
	
		if(GE_ENABLE == gsl_gesture_status && gsl_gesture_flag == 1){
			int tmp_c;
			u8 key_data = 0;
			tmp_c = gsl_obtain_gesture();
			printk("gsl_obtain_gesture():tmp_c=0x%x[%d]\n",tmp_c,test_count++);
			dev_info("gsl_obtain_gesture():tmp_c=0x%x\n",tmp_c);
			switch(tmp_c){
			case (int)'C':
				key_data = KEY_C;
				break;
			case (int)'E':
				key_data = KEY_E;
				break;
			case (int)'W':
				key_data = KEY_W;
				break;
			case (int)'O':
				key_data = KEY_O;
				break;
			case (int)'M':
				key_data = KEY_M;
				break;
			case (int)'Z':
				key_data = KEY_Z;
				break;
			case (int)'V':
				key_data = KEY_V;
				break;
			case (int)'S':
				key_data = KEY_S;
				break;
			case (int)'*':	
				key_data = KEY_POWER;
				break;/* double click */
				case (int)0xa1fa:
				key_data = KEY_F1;
				break;/* right */
			case (int)0xa1fd:
				key_data = KEY_F2;
				break;/* down */
			case (int)0xa1fc:	
				key_data = KEY_F3;
				break;/* up */
			case (int)0xa1fb:	/* left */
				key_data = KEY_F4;
				break;	
			
			}
	
			if(key_data != 0){
				gsl_gesture_c = (char)(tmp_c & 0xff);
				gsl_gesture_status = GE_WAKEUP;
				printk("gsl_obtain_gesture():tmp_c=%c\n",gsl_gesture_c);
				//input_report_key(tpd->dev,key_data,1);
				input_report_key(idev, KEY_POWER,1);
				input_sync(idev);
				//input_report_key(tpd->dev,key_data,0);
				input_report_key(idev,KEY_POWER,0);
				input_sync(idev);
			}
			return;
		}
#endif

	//dev_info("222 finger_num= %d\n",cinfo->finger_num);
	gsl_report_point(idev,cinfo);
	
schedule:
	enable_irq(client->irq);

}

static int gsl_request_input_dev(struct gsl_ts_data *ts_data)

{
	struct input_dev *input_dev = ts_data->input_dev;
	int err;
#if GSL_HAVE_TOUCH_KEY
	int i;
#endif

	/*set input parameter*/	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
#ifdef GSL_REPORT_POINT_SLOT
	__set_bit(EV_REP, input_dev->evbit);
//subin	input_mt_init_slots(input_dev,5);
	input_mt_init_slots(input_dev,5,0);
#else 
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0,5,0,0);
#endif
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
#ifdef TOUCH_VIRTUAL_KEYS
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOME,  input_dev->keybit);    
#endif
#if GSL_HAVE_TOUCH_KEY
	for(i=0;i<GSL_KEY_NUM;i++)
	{
		__set_bit(gsl_key_data[i].key, input_dev->keybit);
	}
#endif
	
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, GSL_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, GSL_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	input_dev->name = GSL_TS_NAME;		//dev_name(&client->dev)

	/*register input device*/
	err = input_register_device(input_dev);
	if (err) {
		goto err_register_input_device_fail;
	}
	return 0;
err_register_input_device_fail:
	input_free_device(input_dev);
	return err;
}

static int gsl_read_version(void)
{
	u8 buf[4] = {0};
	int err= 0;
	buf[0] = 0x03;
	err = gsl_write_interface(ts_data->client, 0xf0, buf, 4);
	if(err < 0) {
		return err;
	}
	gsl_ts_read(ts_data->client, 0x04, buf, 4);
	if(err < 0) {
		return err;
	}
//	ts_data->fw_version = buf[2];
	ts_data->fw_version = buf[0];
	printk("fw version = %d\n", ts_data->fw_version);
//	printk("fw version = %d,%d,%d,%d\n",buf[0],buf[1],buf[2],buf[3]);
//	return 0;
	return ts_data->fw_version;
}

static irqreturn_t gsl_ts_interrupt(int irq, void *dev_id)
{
	struct i2c_client *client = ts_data->client;
	//dev_info("gslX68X_ts_interrupt\n");
	
	disable_irq_nosync(client->irq);

	#ifdef GSL_GESTURE
	if(gsl_gesture_status==GE_ENABLE&&gsl_gesture_flag==1){
		wake_lock_timeout(&gsl_wake_lock, msecs_to_jiffies(2000));
		//dev_info("gsl-jeft\n");
	}
	#endif
	
	if (!work_pending(&ts_data->work)) {
		queue_work(ts_data->wq, &ts_data->work);
	}
	
	return IRQ_HANDLED;
}

/***Begain fw_version sys_class create and read **/
static struct class *firm_ver_class;
static struct device *firm_ver_dev;

static ssize_t firm_ver_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
	int fw_version;
	int fw_o_build_time;
//	int chip_id;
	u8  chip_buf[4];
	int i,err;

	struct i2c_client *client = ts_data->client;

	fw_version = gsl_read_version();		//FW version
	fw_o_build_time = gsl_version_id();		// .o file build id of time
//	chip_id = gsl_compatible_id(client);	//GSL chip id of 3670
	for(i = 0; i < 5; i++) {
		err = gsl_ts_read(client, 0xfc, chip_buf, 4);
		printk("[tp-gsl] 0xfc = {0x%02x%02x%02x%02x}\n", chip_buf[3], chip_buf[2],
			chip_buf[1],chip_buf[0]);
		if(!(err < 0)) {
			err = 1;
			break;
		}
	}
	if(err < 0){
		printk("GSL DEBUG : chip id is not the right one,Please have a check. \n");
	}
	return sprintf ( buf, "fw_version :0x%x \n fw_o_build_time:0x%x \n chip_id(0xfc) : 0x%02x%02x%02x%02x \n",fw_version,fw_o_build_time,chip_buf[3], chip_buf[2],chip_buf[1],chip_buf[0]);

}

static DEVICE_ATTR(firm_ver, 0444, firm_ver_show, NULL);

static void firm_ver_attr_create(void)
{
    firm_ver_class = class_create(THIS_MODULE, "firmware_ver");
    if (IS_ERR(firm_ver_class))
        pr_err("Failed to create class(firm_ver_class)!\n");
    firm_ver_dev = device_create(firm_ver_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firm_ver_dev))
        pr_err("Failed to create device(gt_dclick_dev)!\n");

       // update
    if (device_create_file(firm_ver_dev, &dev_attr_firm_ver) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_firm_ver.attr.name);
}


/***End fw_version sys_class create and read **/


#if defined(CONFIG_FB)
static void gsl_ts_suspend(void)
{
	struct i2c_client *client = ts_data->client;
	u32 tmp;
	int err;
	dev_info("gslX68X_ts_suspend\n");
	/* version information */
	printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

	if(1 == ts_data->updating_fw)
		return;

	if(ts_data->suspended==1){
		printk("GSL %s TP already in suspend status",__func__);
//		pr_notice("GSL %s TP already in suspend status",__func__);
		return;
	}
//	ts_data->suspended = 1;
#if defined(GSL_PROXIMITY_SENSOR)
	bNeedResumeTp = 0;
#endif

#ifdef GSL_PROXIMITY_SENSOR
  if(proximity_enable == 1)
  {
	  flag_tp_down = 1;
	  return ;
  }
#endif
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	printk("GSL %s : GSL_enable = %d \n",__func__,gsl_enabled);
	if(gsl_enabled)
	{
		if(GSL_enable_irq_wake_flag == 0){
			if (device_may_wakeup(&client->dev)){
				err=enable_irq_wake(client->irq);
				pr_notice("GSL : %s --------err --33--- = %d \n",__func__,err);
				if(err)
					printk("%s: GSL TP enable irq wake fail with P-sensor",__func__);
				else
					GSL_enable_irq_wake_flag = 1;
			}
		}
//		flag_tp_down = 1;
		return ;
	}
#endif

	
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif

#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&gsl_timer_check_work);
#endif

/*Guesture Resume*/
#ifdef GSL_GESTURE
		if(gsl_gesture_flag == 1){
			gsl_enter_doze(ts_data);
			return;
		}
#endif

#ifndef GSL_GESTURE
	disable_irq_nosync(client->irq);
	gpio_set_value(ts_data->pdata->reset_gpio, 0);
#endif
//	flag_tp_down = 1;
	ts_data->suspended = 1;
	printk("GSL : %s******Suspend Success ********\n",__func__);
	return;
}

static void gsl_ts_resume(void)
{	
	struct i2c_client *client = ts_data->client;
	int err;
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	u8 buf[4] = {0};
#endif
	
	dev_info("==gslX68X_ts_resume=\n");
	if(1==ts_data->updating_fw){
		ts_data->suspended = 0;
		return;
	}

	/* Proximity Sensor */
	#ifdef GSL_PROXIMITY_SENSOR
		if(proximity_enable == 1)
		{
			if(flag_tp_down == 1)
			{
				flag_tp_down = 0;
				return ;
			}
		}
  	#endif
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	printk("GSL : %s ---------gsl_enabled = %d,ts_data->suspended = %d \n",__func__,gsl_enabled,ts_data->suspended);
	if(gsl_enabled)
	{
#if 0
		if(flag_tp_down == 0)
		{
			return ;
		}
	}
#endif
		if(GSL_enable_irq_wake_flag == 1){
			if (device_may_wakeup(&client->dev)){
				err=disable_irq_wake(client->irq);
				pr_notice("GSL : %s ---gsl_enable = %d --\n",__func__,gsl_enabled);
				if(err)
					printk("%s: GSL TP disable irq wake fail with P-sensor",__func__);
				else
					GSL_enable_irq_wake_flag = 0;
			}
		}
}
#endif

	if(ts_data->suspended == 0){
		pr_notice("GSL %s TP already in resume status\n",__func__);
		printk("GSL %s TP already in resume status\n",__func__);
		return;
	}

	/*Gesture Resume*/
	#ifdef GSL_GESTURE
		if(gsl_gesture_flag == 1){
			gsl_quit_doze(ts_data);
		}
	#endif


	gpio_set_value(ts_data->pdata->reset_gpio, 1);
	msleep(20);
	gsl_reset_core(client);

	/*Gesture Resume*/
	#ifdef GSL_GESTURE	
		#ifdef GSL_ALG_ID
			gsl_DataInit(gsl_config_data_id);
		#endif
	#endif
	
	gsl_start_core(client);
	msleep(20);
	check_mem_data(client);
	enable_irq(client->irq);
	
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
//	flag_tp_down = 0;
	if(gsl_enabled)
	{
		buf[3] = 0;
		buf[2] = 0;
		buf[1] = 0;
		buf[0] = 0x4;
		gsl_write_interface(ts_data->client,0xf0,buf,4);
		buf[3] = 0;
		buf[2] = 0;
		buf[1] = 0;
		buf[0] = 0x2;
		gsl_write_interface(ts_data->client,0x0,buf,4);
	}
#endif
	
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
	
#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif

	ts_data->suspended = 0;
	printk("GSL : %s ***Resume Success ****\n",__func__);
	return;

}

static void gsl_ts_resume_work(struct work_struct *work)
{
	gsl_ts_resume();
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	
	if (evdata && evdata->data && event == FB_EVENT_BLANK ){
		blank = evdata->data;
		dev_info("fb_notifier_callback blank=%d\n",*blank);
		if (*blank == FB_BLANK_UNBLANK) {			
			if (!work_pending(&ts_data->resume_work)){
//				schedule_work(&ts_data->resume_work);
				schedule_work(&ts_data->fb_notify_work);
			}
		}else if (*blank == FB_BLANK_POWERDOWN) {
			cancel_work_sync(&ts_data->resume_work);
			flush_work(&ts_data->fb_notify_work);
			gsl_ts_suspend();
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void gsl_early_suspend(struct early_suspend *handler)
{
	u32 tmp;
	struct i2c_client *client = ts_data->client;
	dev_info("==gslX68X_ts_suspend=\n");
	//version info
	printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

	if(1==ts_data->updating_fw)
		return;

	ts_data->suspended = 1;

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif

#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&gsl_timer_check_work);
#endif	

	disable_irq_nosync(client->irq);
	gpio_set_value(GSL_RST_GPIO_NUM, 0);
}

static void gsl_early_resume(struct early_suspend *handler)
{	
	struct i2c_client *client = ts_data->client;
	dev_info("==gslX68X_ts_resume=\n");
	if(1==ts_data->updating_fw){
		ts_data->suspended = 0;
		return;
	}

	gpio_set_value(GSL_RST_GPIO_NUM, 1);
	msleep(20);
	gsl_reset_core(client);
	gsl_start_core(client);
	msleep(20);
	check_mem_data(client);
	enable_irq(client->irq);
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
	
#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
	
	ts_data->suspended = 0;

}
#endif

#if defined(CONFIG_FB)
static int gsl_get_dt_coords(struct device *dev, char *name,
				struct gsl_ts_platform_data *pdata)
{
	u32 coords[GSL_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != GSL_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "gsl,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "gsl,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int gsl_parse_dt(struct device *dev, struct gsl_ts_platform_data *pdata)
{

	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val;

	pdata->name = "gsl";
	rc = of_property_read_string(np, "gsl,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = gsl_get_dt_coords(dev, "gsl,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = gsl_get_dt_coords(dev, "gsl,display-coords", pdata);
	if (rc)
		return rc;

	rc = of_property_read_u32(np, "gsl,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_reset_delay_ms = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "gsl,post-hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->post_hard_reset_delay_ms = temp_val;
	else
		return rc;

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "gsl,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "gsl,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	rc = of_property_read_u32(np, "gsl,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	prop = of_find_property(np, "gsl,button-map", NULL);
	if (prop) {
		pdata->num_buttons = prop->length / sizeof(temp_val);
		if (pdata->num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"gsl,button-map", pdata->button_map,
			pdata->num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int gsl_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
	return -ENODEV;
}
#endif

//su
#if 0
static void gsl_ts_register_productinfo(void)
{
	// format as flow: version:0x01 Module id:0x57
	char deviceinfo[64];
    sprintf(deviceinfo, "FW version:0x%2x Module id:0x%2x",
                        ts_data->fw_version, gsl_cfg_index);

	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);
}
#endif
//su
//su
#if 0
{
	return 1;
}


static int factory_get_ic_fw_version(struct device *dev, char *buf)
{	
	return sprintf(buf, "0x%02X\n", ts_data->fw_version);
}

static int factory_get_fs_fw_version(struct device *dev, char *buf)
{
	return sprintf(buf, "0x%02X\n", ts_data->fw_version);
}

static int factory_get_module_id(struct device *dev, char *buf)
{
	return sprintf(buf, "0x%02X\n", gsl_cfg_index);
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

static int factory_get_rawdata(struct device *dev, char *buf)
{
	return sprintf(buf, "%s\n", "NOT SUPPORTED");
}

static int factory_get_rawdata_info(struct device *dev, char *buf)
{
	return sprintf(buf, "%s\n", "NOT SUPPORTED");
}
#endif 
//su

//su
#if 0
static int factory_ts_func_test_register(void)
{
	ts_data->ts_test_dev.dev = &ts_data->client->dev;
	ts_data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	ts_data->ts_test_dev.get_fs_fw_version = factory_get_fs_fw_version;
	ts_data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	ts_data->ts_test_dev.get_module_id = factory_get_module_id;
	ts_data->ts_test_dev.get_rawdata = factory_get_rawdata;
	ts_data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info;	
	ts_data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test; 

	register_ts_func_test_device(&ts_data->ts_test_dev);
	return 0;
}
#endif
//su

static int gsl_psensor_input_register(struct i2c_client *pClient)
{
	s32 nRetVal = 0;

	printk("*** %s() ***\n", __func__);

	proximity_dev= input_allocate_device();
	if (proximity_dev == NULL)
//	if (vps->virtualdevice == NULL)
	{
		printk("*** input device allocation failed ***\n");
		return -ENOMEM;
	}
	proximity_dev->name = VPS_NAME;
	proximity_dev->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	set_bit(EV_ABS, proximity_dev->evbit);
	set_bit(ABS_DISTANCE, proximity_dev->absbit);
	input_set_abs_params(proximity_dev, ABS_DISTANCE, 0, 1, 0, 0);
	nRetVal = input_register_device(proximity_dev);
	if (nRetVal < 0)
	{
		printk("*** Unable to register virtual P-sensor input device ***\n");
		return nRetVal;
	}
	vps_cdev = virtual_sensors_proximity_cdev;
	vps_cdev.sensors_enable = vps_set_enable;
	vps_cdev.sensors_poll_delay = NULL;
	nRetVal = sensors_classdev_register(&pClient->dev, &vps_cdev);
	if (nRetVal) {
		printk("%s: Unable to register to sensors class: %d\n",__func__, nRetVal);
	return nRetVal;
	}

	return 0;
}

static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int err = 0;
	int ret = 0;
	struct input_dev *input_dev;
	
	#ifdef GSL_PROXIMITY_SENSOR
			struct input_dev *input_dev_ps;
	#endif

	struct gsl_ts_platform_data *pdata;
	dev_info("%s\n",__func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct gsl_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = gsl_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "Failed to parse dt\n");
			err = -ENOMEM;
			goto exit_parse_dt_err;
		}
	}
	else
		pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
        dev_err(&client->dev, "I2c doesn't work\n");
		goto exit_check_functionality_failed;
	}

	ts_data = kzalloc(sizeof(struct gsl_ts_data), GFP_KERNEL);
	if (ts_data==NULL) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	ts_data->suspended = 0;
	ts_data->updating_fw = 0;
	ts_data->gsl_up_flag = 0;
	ts_data->gsl_point_state = 0;
#if GSL_HAVE_TOUCH_KEY
	ts_data->gsl_key_state = 0;
#endif	
	ts_data->cinfo = kzalloc(sizeof(struct gsl_touch_info),GFP_KERNEL);
	if(ts_data->cinfo == NULL) {
		err = -ENOMEM;
		goto exit_alloc_cinfo_failed;
	}
	mutex_init(&ts_data->gsl_i2c_lock);

	/*allocate input device*/
	dev_info("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_allocate_input_device_fail;
	}

	input_dev->name = client->name;
	input_dev->phys = "I2C";
	input_dev->dev.parent = &client->dev;
	input_dev->id.bustype = BUS_I2C;

	ts_data->input_dev = input_dev;
	ts_data->client = client;
	ts_data->pdata = pdata;
	i2c_set_clientdata(client, ts_data);
	dev_info("I2C addr=%x\n", client->addr);	

	err = gsl_ts_power_init(ts_data, true);
	if (err) {
		dev_err(&client->dev, "Silead power init failed\n");
		goto exit_power_init_err;
	}

	err = gsl_ts_power_on(ts_data, true);
	if (err) {
		dev_err(&client->dev, "Silead power on failed\n");
		goto exit_power_on_err;
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
			err = gpio_request(pdata->irq_gpio, GSL_IRQ_NAME);
			if (err) {
				dev_err(&client->dev, "irq gpio request failed");
				goto exit_request_irq_gpio_err;
			}
			err = gpio_direction_input(pdata->irq_gpio);
			if (err) {
				dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
			goto exit_set_irq_gpio_err;
			}
		}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, GSL_RST_NAME);
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto exit_request_reset_gpio_err;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto exit_set_reset_gpio_err;
		}
		msleep(20);
		gpio_set_value_cansleep(pdata->reset_gpio, 1);
	}

	/* make sure CTP already finish startup process */
	msleep(100);

	err = gsl_read_version();
	if(err < 0) {
		goto exit_i2c_transfer_fail; 
	}
	
	err = gsl_compatible_id(client);
	if(err < 0) {
		goto exit_i2c_transfer_fail; 
	}

#ifdef GSL_COMPATIBLE_GPIO
	gsl_gpio_idt_tp(client);
#endif
	/*request input system*/	
	err = gsl_request_input_dev(ts_data);
	if(err < 0) {
		goto exit_i2c_transfer_fail;	
	}

	/*register early suspend*/
	dev_info("==register_early_suspend =\n");
	
	#if defined(CONFIG_FB)
	INIT_WORK(&ts_data->fb_notify_work, gsl_ts_resume_work);
//	INIT_WORK(&ts_data->resume_work, gsl_ts_resume_work);
	ts_data->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts_data->fb_notif);
	if (err)
		dev_err(&ts_data->client->dev,
			"Unable to register fb_notifier: %d\n",
			err);
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts_data->pm.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts_data->pm.suspend = gsl_early_suspend;
	ts_data->pm.resume = gsl_early_resume;
	register_early_suspend(&ts_data->pm);
	#endif

	/*init work queue*/
	INIT_WORK(&ts_data->work, gsl_report_work);
	ts_data->wq = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ts_data->wq) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	/*request irq */
	err = request_irq(client->irq, gsl_ts_interrupt, IRQF_TRIGGER_FALLING, client->name, ts_data);
	if (err < 0) {
		dev_err(&client->dev, "gslX68X_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	
	disable_irq_nosync(client->irq);

	/*gesture resume*/
#ifdef GSL_GESTURE
	gsl_FunIICRead(gsl_read_oneframe_data);
#endif
	
	/*gsl of software init*/
	gsl_sw_init(client);
	msleep(20);
	check_mem_data(client);

#ifdef TOUCH_VIRTUAL_KEYS
	gsl_ts_virtual_keys_init();
#endif

#ifdef TPD_PROC_DEBUG
	gsl_config_proc = proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
	
	printk("GSL *********%s******gsl_config_proc**\n",__func__);

if (gsl_config_proc == NULL)
	{
		printk("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		printk("create proc entry %s success\n", GSL_CONFIG_PROC_FILE);
		//gsl_config_proc->read_proc = gsl_config_read_proc;
		//gsl_config_proc->write_proc = gsl_config_write_proc;
	}

	gsl_proc_flag = 0;
#endif
	
#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
	gsl_timer_workqueue = create_workqueue("gsl_timer_check");
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
	err = sysfs_create_group(&client->dev.kobj,&gsl_attr_group);

#ifdef GSL_GESTURE
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_POWER);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_C);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_E);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_O);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_W);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_M);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_Z);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_V);
		input_set_capability(ts_data->input_dev, EV_KEY, KEY_S);
#endif

#ifdef GSL_PROXIMITY_SENSOR
		input_dev_ps = input_allocate_device();
		if (!input_dev_ps) {
			printk("%s: Failed to allocate input device ps\n", __func__);
		}

		ts_data->input_dev_ps = input_dev_ps;
		set_bit(EV_ABS, input_dev_ps->evbit);
		input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
		input_dev_ps->name = "proximity";
		err = input_register_device(input_dev_ps);

		ts_data->ps_cdev = sensors_proximity_cdev;
		ts_data->ps_cdev.sensors_enable = psensor_ps_set_enable;
		ts_data->ps_cdev.sensors_ptoll_delay = NULL;
		err = sensors_classdev_register(&client->dev, &ts_data->ps_cdev);
		gsl_gain_psensor_data(ts_data->client);
#endif

#if defined(GSL_PROXIMITY_SENSOR)
	   wake_lock_init(&ps_wake_lock, WAKE_LOCK_SUSPEND, "ps_wakelock");
#endif

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	//proximity_dev = input_allocate_device();
	ret  = gsl_psensor_input_register(ts_data->client);
	gsl_gain_psensor_data(ts_data->client);
	input_report_abs(proximity_dev, ABS_DISTANCE, 1);
	input_sync(proximity_dev);
	sys_device_create();
	wake_lock_init(&GSL_wakelock,WAKE_LOCK_SUSPEND, "GSL_wakelock");
#endif
	   
#ifdef GSL_TEST_TP
		create_ctp_proc();
#endif

	gsl_read_version();
	firm_ver_attr_create();
//su
#if 0
	gsl_ts_register_productinfo();
	factory_ts_func_test_register();
#endif
//su
	enable_irq(client->irq);

#ifdef GSL_GESTURE
	enable_irq_wake(client->irq);
#endif

	dev_info("%s: ==probe over =\n",__func__);
	return 0;

exit_irq_request_failed:
	cancel_work_sync(&ts_data->work);
	destroy_workqueue(ts_data->wq);
exit_create_singlethread:
	#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts_data->fb_notif))
		dev_err(&client->dev,
			"Error occurred while unregistering fb_notifier.\n");

	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts_data->pm);
	#endif
 
	input_unregister_device(ts_data->input_dev);
	input_free_device(ts_data->input_dev);
exit_i2c_transfer_fail:
exit_set_reset_gpio_err:
	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);
exit_request_reset_gpio_err:
exit_set_irq_gpio_err:
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
exit_request_irq_gpio_err:
	gsl_ts_power_on(ts_data, false);
exit_power_on_err:
	gsl_ts_power_init(ts_data, false);
exit_power_init_err:
err_allocate_input_device_fail:
	i2c_set_clientdata(client, NULL);
	kfree(ts_data->cinfo);
exit_alloc_cinfo_failed:
	kfree(ts_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
exit_parse_dt_err:
	//kfree(pdata);

	return err;
}

static int  gsl_ts_remove(struct i2c_client *client)
{

	dev_info("==gslX68X_ts_remove=\n");

#ifdef TPD_PROC_DEBUG
	if(gsl_config_proc!=NULL)
		remove_proc_entry(GSL_CONFIG_PROC_FILE, NULL);
#endif
	
	if (fb_unregister_client(&ts_data->fb_notif))
			dev_err(&client->dev,"Error occurred while unregistering fb_notifier.\n");

	free_irq(client->irq,ts_data);
	input_unregister_device(ts_data->input_dev);
	input_free_device(ts_data->input_dev);
	gpio_free(GSL_RST_GPIO_NUM);
	gpio_free(GSL_IRQ_GPIO_NUM);
	
	cancel_work_sync(&ts_data->work);
	destroy_workqueue(ts_data->wq);
	i2c_set_clientdata(client, NULL);
	//sprd_free_gpio_irq(client->irq);
	kfree(ts_data->cinfo);
	kfree(ts_data);
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	wake_lock_destroy(&GSL_wakelock);
#endif
	
	gsl_ts_power_on(ts_data, false);

	gsl_ts_power_init(ts_data, false);

	return 0;
}


static const struct i2c_device_id gsl_ts_id[] = {
	{ GSL_TS_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, gsl_ts_id);


#if defined(CONFIG_FB)
static struct of_device_id gsl_match_table[] = {
	{ .compatible = "silead,gsl-tp",},
	{ },
};
#else
#define gsl_match_table NULL
#endif

static struct i2c_driver gsl_ts_driver = {
	.driver = {
		.name = GSL_TS_NAME,
        .owner    = THIS_MODULE,
		.of_match_table = gsl_match_table,
	},
	.probe = gsl_ts_probe,
	.remove = gsl_ts_remove,
	.id_table	= gsl_ts_id,
};
 

module_i2c_driver(gsl_ts_driver);

MODULE_AUTHOR("sileadinc");
MODULE_DESCRIPTION("GSL Series Driver");
MODULE_LICENSE("GPL");

