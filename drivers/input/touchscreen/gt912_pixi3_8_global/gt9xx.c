/*
 * Goodix GT9xx touchscreen driver
 *
 * Copyright  (C)  2010 - 2014 Goodix. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.4
 * Release Date: 2014/11/28
 */

#include <linux/irq.h>
#include "gt9xx.h"
#ifdef CONFIG_SENSORS_HALL
#include "../../misc/hall/hall.h"
#endif

#if GTP_ICS_SLOT_REPORT
    #include <linux/input/mt.h>
#endif

/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
#define GOODIX_PINCTRL_STATE_SLEEP "gt9xx_int_suspend"
#define GOODIX_PINCTRL_STATE_DEFAULT "gt9xx_int_default"
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

static const char *goodix_ts_name = "goodix-ts";
static const char *goodix_input_phys = "input/ts";
static struct workqueue_struct *goodix_wq;
struct i2c_client * gtp_912_i2c_connect_client = NULL;
int gtp_rst_gpio;
int gtp_int_gpio;
u8 gt912_config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
    static const u16 touch_key_array[] = GTP_KEY_TAB;
    #define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
    static const int  key_codes[] = {KEY_HOME, KEY_BACK, KEY_MENU, KEY_SEARCH};
    static const char *key_names[] = {"Key_Home", "Key_Back", "Key_Menu", "Key_Search"};
#endif

#endif

//[FEATURE]-Add-begin by TCTNB.qw  2015/04/27  FR990180  show fw version for app.
static void firm_ver_attr_create(void);
static ssize_t firm_ver_show ( struct device *dev,struct device_attribute *attr, char *buf );
//[FEATURE]-Add-end by TCTNB.qw  2015/04/27  FR990180  show fw version for app.

//[NEW FEATURE] add for gesture switch attr file
#if (GTP_GESTURE_WAKEUP || GTP_PALM_SUSPEND)
static struct class *gesture_switch_class;
static struct device *gesture_switch_dev;
static void gesture_switch_attr_create(void);
#endif

//ADD FOR GESTURE SWITCH 2015/06/29/
#if GTP_GESTURE_WAKEUP

#define    DOZE_SWITCH_ON 1
#define    DOZE_SWITCH_OFF 0

static bool doze_switch_flag = 1;
static ssize_t doze_switch_att_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int switch_flag;
    if(sscanf(buf, "%d", &switch_flag) != 1)
        return -EINVAL;
    if(switch_flag == DOZE_SWITCH_ON)
    {
        doze_switch_flag = DOZE_SWITCH_ON;
    }
    else if(switch_flag == DOZE_SWITCH_OFF)
    {
        doze_switch_flag = DOZE_SWITCH_OFF;
    }
    else
        return -EINVAL;
    return count;
}

static ssize_t doze_switch_att_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    return sprintf(buf,"%d\n",doze_switch_flag);
}

static DEVICE_ATTR(doze_switch,0666,doze_switch_att_show,doze_switch_att_store);

#endif
//END FOR GESTURE SWITCH 2015/06/29/

//ADD PALM SUSPEND SWITCH 2015/06/30
#if GTP_PALM_SUSPEND
#define    PALM_SWITCH_ON 1
#define    PALM_SWITCH_OFF 0
static bool palm_switch_flag = 1;
static ssize_t palm_switch_att_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int palm_flag;
    if(sscanf(buf, "%d", &palm_flag) != 1)
        return -EINVAL;
    if(palm_flag == PALM_SWITCH_ON)
    {
        palm_switch_flag = PALM_SWITCH_ON;
    }
    else if(palm_flag == PALM_SWITCH_OFF)
    {
        palm_switch_flag = PALM_SWITCH_OFF;
    }
    else
        return -EINVAL;
    return count;
}

static ssize_t palm_switch_att_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    return sprintf(buf,"%d\n",palm_switch_flag);
}

static DEVICE_ATTR(palm_switch,0666,palm_switch_att_show,palm_switch_att_store);
#endif
//ADD PALM SUSPEND SWITCH END

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_912_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_912_send_cfg(struct i2c_client *client);
static void gtp_int_sync(s32 ms);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};
static int gtp_register_powermanger(struct goodix_ts_data *ts);
static int gtp_unregister_powermanger(struct goodix_ts_data *ts);

#if GTP_CREATE_WR_NODE
extern s32 gt912_init_wr_node(struct i2c_client*);
extern void gt912_uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_912_esd_switch(struct i2c_client *, s32);
#endif

//*********** For GT9XXF Start **********//
#if GTP_COMPATIBLE_MODE
extern s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);
extern int gup_reload_fw_dsp(void *dir, u8 down_mode);

void gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);

#endif
//********** For GT9XXF End **********//

#ifdef CONFIG_SENSORS_HALL
extern struct hall_data * hall;
#endif

/*ADD Begin by TCTNB-QW	2015-04-23	,  FR978057 */
#if 0
static u8 palm_lock_flag = 0;
u8	g_palm_lock_switch=0;
#endif

#if CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#define VPS_NAME "virtual-proximity"
struct virtualpsensor *vps;
static struct sensors_classdev virtual_sensors_proximity_cdev = {
	.name = VPS_NAME,
	.vendor = "NULL",
	.version = 1,
	.handle = SENSORS_VIRTUAL_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_VIRTUAL_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "8.0",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static void tp_prox_sensor_enable(struct i2c_client *client,int enable);
#endif
/*ADD End by TCTNB-QW		2015-04-23	,  FR978057 */

#if GTP_GESTURE_WAKEUP
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif

#ifdef CONFIG_OF
int gtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid);
#endif

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
      2: succeed, otherwise: failed
*********************************************************/
static s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif

    #if GTP_GESTURE_WAKEUP
        if(doze_switch_flag)
        {
            // reset chip would quit doze mode
            if (DOZE_ENABLED == doze_status)
            {
                return ret;
            }
        }
    #endif
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_912_reset_guitar(client, 10);
        }
    }
    return ret;
}



/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif

    #if GTP_GESTURE_WAKEUP
        if(doze_switch_flag)
        {
            if (DOZE_ENABLED == doze_status)
            {
                return ret;
            }
        }
    #endif
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_912_reset_guitar(client, 10);
        }
    }
    return ret;
}


/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_912_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;
    int ret =0;

    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        ret = gtp_i2c_read(client, buf, len + 2);
        if (ret < 0)
        return FAIL;

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        ret = gtp_i2c_read(client, confirm_buf, len + 2);
        if (ret < 0)
        return FAIL;
        if (!memcmp(buf, confirm_buf, len+2))
        {   GTP_ERROR("gtp_912_i2c_read_dbl_check return success \n");
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }
    GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/

s32 gtp_912_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->pnl_init_error)
    {
        GTP_INFO("Error occured in init_panel, no config sent");
        return 0;
    }

    GTP_INFO("Driver send config.");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, gt912_config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif
    return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_912_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1;
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_912_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable)
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 1);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->input_dev);
#endif

    GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
    GTP_DEBUG("Touch id[%2d] release!", id);
#else
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
}

#if GTP_WITH_PEN

static void gtp_pen_init(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    GTP_INFO("Request input device for pen/stylus.");

    ts->pen_dev = input_allocate_device();
    if (ts->pen_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device for pen/stylus.");
        return;
    }

    ts->pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

#if GTP_ICS_SLOT_REPORT
    input_mt_init_slots(ts->pen_dev, 16);               //
#else
    ts->pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

    set_bit(BTN_TOOL_PEN, ts->pen_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, ts->pen_dev->propbit);
    //set_bit(INPUT_PROP_POINTER, ts->pen_dev->propbit);

#if GTP_PEN_HAVE_BUTTON
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS);
    input_set_capability(ts->pen_dev, EV_KEY, BTN_STYLUS2);
#endif

    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    ts->pen_dev->name = "goodix-pen";
    ts->pen_dev->id.bustype = BUS_I2C;

    ret = input_register_device(ts->pen_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->pen_dev->name);
        return;
    }
}

static void gtp_pen_down(s32 x, s32 y, s32 w, s32 id)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(gtp_912_i2c_connect_client);

#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

    input_report_key(ts->pen_dev, BTN_TOOL_PEN, 1);
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->pen_dev, id);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
    input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
#else
    input_report_key(ts->pen_dev, BTN_TOUCH, 1);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->pen_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->pen_dev, ABS_MT_PRESSURE, w);
    input_report_abs(ts->pen_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->pen_dev);
#endif
    GTP_DEBUG("(%d)(%d, %d)[%d]", id, x, y, w);
}

static void gtp_pen_up(s32 id)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(gtp_912_i2c_connect_client);

    input_report_key(ts->pen_dev, BTN_TOOL_PEN, 0);

#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->pen_dev, id);
    input_report_abs(ts->pen_dev, ABS_MT_TRACKING_ID, -1);
#else

    input_report_key(ts->pen_dev, BTN_TOUCH, 0);
#endif

}
#endif

//[FEATURE] Add  begin by TCT-NB-QW 	2015/04/23  FR978057
#if CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
static void tp_prox_sensor_enable(struct i2c_client *client, int enable)
{
    if(client == NULL)
		return;
	if(enable)
		g_palm_lock_switch=1;
	else
		g_palm_lock_switch=0;
}

static int vps_set_enable(struct sensors_classdev *sensors_cdev,unsigned int enable)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(gtp_912_i2c_connect_client);
	vps->vps_enabled = enable ? 1 : 0;
	tp_prox_sensor_enable(ts->client, enable);
	return 0;
}

static int virtual_psensor_input_register(struct i2c_client *pClient)
{
	s32 nRetVal = 0;

	printk("*** %s() ***\n", __func__);
	vps = kzalloc(sizeof(struct virtualpsensor), GFP_KERNEL);
	vps->client = pClient;
	//i2c_set_clientdata(pClient, vps);

	vps->virtualdevice= input_allocate_device();
	if (vps->virtualdevice == NULL)
	{
		printk("*** input device allocation failed ***\n");
		return -ENOMEM;
	}
	vps->virtualdevice->name = VPS_NAME;
	vps->virtualdevice->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	set_bit(EV_ABS, vps->virtualdevice->evbit);
	set_bit(ABS_DISTANCE, vps->virtualdevice->absbit);
	input_set_abs_params(vps->virtualdevice, ABS_DISTANCE, 0, 1, 0, 0);
	nRetVal = input_register_device(vps->virtualdevice);
	if (nRetVal < 0)
	{
		printk("*** Unable to register virtual P-sensor input device ***\n");
		return nRetVal;
	}
	vps->vps_cdev = virtual_sensors_proximity_cdev;
	vps->vps_cdev.sensors_enable = vps_set_enable;
	vps->vps_cdev.sensors_poll_delay = NULL;
	nRetVal = sensors_classdev_register(&pClient->dev, &vps->vps_cdev);
	if (nRetVal) {
		printk("%s: Unable to register to sensors class: %d\n",__func__, nRetVal);
	return nRetVal;
	}

	return 0;
}
#endif
//[FEATURE] Add  end by TCT-NB-QW 	2015/04/23  FR978057


/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
#if GTP_WITH_PEN
    u8 pen_active = 0;
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;

#if GTP_COMPATIBLE_MODE
    u8 rqst_buf[3] = {0x80, 0x43};  // for GT9XXF
#endif

#if GTP_GESTURE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#endif

    GTP_DEBUG_FUNC();
    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
#if GTP_GESTURE_WAKEUP
    if (DOZE_ENABLED == doze_status)
    {
        ret = gtp_i2c_read(gtp_912_i2c_connect_client, doze_buf, 3);
        GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
        if (ret > 0)
        {
            if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
                (doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') ||
                (doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
                (doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') ||
                (doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
                (doze_buf[2] == 0x5E) /* ^ */
                )
            {
                if (doze_buf[2] != 0x5E)
                {
                    GTP_INFO("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
                }
                else
                {
                    GTP_INFO("Wakeup by gesture(^), light up the screen!");
                }
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(gtp_912_i2c_connect_client, doze_buf, 3);
			}
			else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
				(doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
            {
                char *direction[4] = {"Right", "Down", "Up", "Left"};
                u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;

                GTP_INFO("%s slide to light up the screen!", direction[type]);
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(gtp_912_i2c_connect_client, doze_buf, 3);
            }
            else if (0xCC == doze_buf[2])
            {
                GTP_INFO("Double click to light up the screen!");
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(gtp_912_i2c_connect_client, doze_buf, 3);
            }
            else
            {
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(gtp_912_i2c_connect_client, doze_buf, 3);
                gtp_enter_doze(ts);
            }
        }
        if (ts->use_irq)
        {
            gtp_912_irq_enable(ts);
        }
        return;
    }
#endif

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        if (ts->use_irq)
        {
            gtp_912_irq_enable(ts);
        }
        return;
    }

    finger = point_data[GTP_ADDR_LENGTH];
//Add begin by TCTNB-QW	2015/04/23		FR978057
#if CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(vps->vps_enabled)
	{
	if(g_palm_lock_switch == 0)
		{
			tp_prox_sensor_enable(ts->client, 1);
		}
		if((finger&(1<<6))&&(0 == palm_lock_flag))
		{
			printk("*************************************distance cat********************\n");
			palm_lock_flag=1;
			input_report_abs(vps->virtualdevice, ABS_DISTANCE, 0);
			input_sync(vps->virtualdevice);
		}
		else if((0==(finger&(1<<6)))&&(1 == palm_lock_flag))
		{
			printk("***********************************distance not cat******************** \n");
			palm_lock_flag=0;
			input_report_abs(vps->virtualdevice, ABS_DISTANCE, 1);
			input_sync(vps->virtualdevice);
		}
	}
#endif
//Add end by TCTNB-QW	2015/04/23		FR978057

#if GTP_COMPATIBLE_MODE
    // GT9XXF reques event
    if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type))     // request arrived
    {
        ret = gtp_i2c_read(ts->client, rqst_buf, 3);
        if (ret < 0)
        {
           GTP_ERROR("Read request status error!");
           goto exit_work_func;
        }

        switch (rqst_buf[2])
        {
        case GTP_RQST_CONFIG:
            GTP_INFO("Request for config.");
            ret = gtp_912_send_cfg(ts->client);
            if (ret < 0)
            {
                GTP_ERROR("Request for config unresponded!");
            }
            else
            {
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                GTP_INFO("Request for config responded!");
            }
            break;

        case GTP_RQST_BAK_REF:
            GTP_INFO("Request for backup reference.");
            ts->rqst_processing = 1;
            ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
            if (SUCCESS == ret)
            {
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                ts->rqst_processing = 0;
                GTP_INFO("Request for backup reference responded!");
            }
            else
            {
                GTP_ERROR("Requeset for backup reference unresponed!");
            }
            break;

        case GTP_RQST_RESET:
            GTP_INFO("Request for reset.");
            gtp_recovery_reset(ts->client);
            break;

        case GTP_RQST_MAIN_CLOCK:
            GTP_INFO("Request for main clock.");
            ts->rqst_processing = 1;
            ret = gtp_main_clk_proc(ts);
            if (FAIL == ret)
            {
                GTP_ERROR("Request for main clock unresponded!");
            }
            else
            {
                GTP_INFO("Request for main clock responded!");
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                ts->rqst_processing = 0;
                ts->clk_chk_fs_times = 0;
            }
            break;
        default:
            GTP_INFO("Undefined request: 0x%02X", rqst_buf[2]);
            rqst_buf[2] = GTP_RQST_RESPONDED;
            gtp_i2c_write(ts->client, rqst_buf, 3);
            break;
        }
    }
#endif
    if (finger == 0x00)
    {
        if (ts->use_irq)
        {
            gtp_912_irq_enable(ts);
        }
        return;
    }

    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

#if GTP_PALM_SUSPEND
    if(palm_switch_flag)
    {
        if (finger & 0x40) // large touch was detected
        {
            if (ts->gtp_is_suspend)
                goto exit_work_func;
	    GTP_INFO("Large touch was detected!");
            input_report_key(ts->input_dev, KEY_POWER, 1);
            input_sync(ts->input_dev);
            input_report_key(ts->input_dev, KEY_POWER, 0);
            input_sync(ts->input_dev);
            ts->gtp_is_suspend = 1;

            goto exit_work_func;
        }
    }
#endif

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
    key_value = point_data[3 + 8 * touch_num];

    if(key_value || pre_key)
    {
    #if GTP_PEN_HAVE_BUTTON
        if (key_value == 0x40)
        {
            GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else if (key_value == 0x10)
        {
            GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 1);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            pen_active = 1;
        }
        else if (key_value == 0x20)
        {
            GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 1);
            pen_active = 1;
        }
        else
        {
            GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
            input_report_key(ts->pen_dev, BTN_STYLUS, 0);
            input_report_key(ts->pen_dev, BTN_STYLUS2, 0);
            if ( (pre_key == 0x40) || (pre_key == 0x20) ||
                 (pre_key == 0x10)
               )
            {
                pen_active = 1;
            }
        }
        if (pen_active)
        {
            touch_num = 0;      // shield pen point
            //pre_touch = 0;    // clear last pen status
        }
    #endif

    #if GTP_HAVE_TOUCH_KEY
        if (!pre_touch)
        {
            for (i = 0; i < GTP_MAX_KEY_NUM; i++)
            {
            #if GTP_DEBUG_ON
                for (ret = 0; ret < 4; ++ret)
                {
                    if (key_codes[ret] == touch_key_array[i])
                    {
                        GTP_DEBUG("Key: %s %s", key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
                        break;
                    }
                }
            #endif
                input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));
            }
            touch_num = 0;  // shield fingers
        }
    #endif
    }
#endif
    pre_key = key_value;

    //GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
    if (pre_pen && (touch_num == 0))
    {
        GTP_DEBUG("Pen touch UP(Slot)!");
        gtp_pen_up(0);
        pen_active = 1;
        pre_pen = 0;
    }
#endif
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;
        u8 report_num = 0;
        coor_data = &point_data[3];

        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;

        #if GTP_WITH_PEN
            id = coor_data[pos];
            if ((id & 0x80))
            {
                GTP_DEBUG("Pen touch DOWN(Slot)!");
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                gtp_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pre_touch = 0;
                pen_active = 1;
            }
        #endif

            touch_index |= (0x01<<id);
        }

        GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",id, touch_index,pre_touch);
        for (i = 0; i < GTP_MAX_TOUCH; i++)
        {
        #if GTP_WITH_PEN
            if (pre_pen == 1)
            {
                break;
            }
        #endif

            if ((touch_index & (0x01<<i)))
            {
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                gtp_touch_down(ts, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;

                report_num++;
                if (report_num < touch_num)
                {
                    pos += 8;
                    id = coor_data[pos] & 0x0F;
                    touch_index |= (0x01<<id);
                }
            }
            else
            {
                gtp_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }
#else

    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | (coor_data[2] << 8);
            input_y  = coor_data[3] | (coor_data[4] << 8);
            input_w  = coor_data[5] | (coor_data[6] << 8);

        #if GTP_WITH_PEN
            id = coor_data[0];
            if (id & 0x80)
            {
                GTP_DEBUG("Pen touch DOWN!");
                gtp_pen_down(input_x, input_y, input_w, 0);
                pre_pen = 1;
                pen_active = 1;
                break;
            }
            else
        #endif
            {
                gtp_touch_down(ts, id, input_x, input_y, input_w);
            }
        }
    }
    else if (pre_touch)
    {
    #if GTP_WITH_PEN
        if (pre_pen == 1)
        {
            GTP_DEBUG("Pen touch UP!");
            gtp_pen_up(0);
            pre_pen = 0;
            pen_active = 1;
        }
        else
    #endif
        {
            GTP_DEBUG("Touch Release!");
            gtp_touch_up(ts, 0);
        }
    }

    pre_touch = touch_num;
#endif

#if GTP_WITH_PEN
    if (pen_active)
    {
        pen_active = 0;
        input_sync(ts->pen_dev);
    }
    else
#endif
    {
        input_sync(ts->input_dev);
    }

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd error!");
        }
    }
    if (ts->use_irq)
    {
        gtp_912_irq_enable(ts);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode.
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();

    gtp_912_irq_disable(ts);

    queue_work(goodix_wq, &ts->work);

    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    GTP_GPIO_OUTPUT(gtp_int_gpio, 0);
    msleep(ms);
    GTP_GPIO_AS_INT(gtp_int_gpio);
}


/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gtp_912_reset_guitar(struct i2c_client *client, s32 ms)
{
#if GTP_COMPATIBLE_MODE
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
#endif

    GTP_DEBUG_FUNC();
    GTP_INFO("Guitar reset");
    GTP_GPIO_OUTPUT(gtp_rst_gpio, 0);   // begin select I2C slave addr
    msleep(ms);                         // T2: > 10ms
    // HIGH: 0x28/0x29, LOW: 0xBA/0xBB
    GTP_GPIO_OUTPUT(gtp_int_gpio, client->addr == 0x14);

    msleep(2);                          // T3: > 100us
    GTP_GPIO_OUTPUT(gtp_rst_gpio, 1);

    msleep(6);                          // T4: > 5ms

    GTP_GPIO_AS_INPUT(gtp_rst_gpio);    // end select I2C slave addr

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        return;
    }
#endif

    gtp_int_sync(50);
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(client);
#endif
}


/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

    GTP_DEBUG("Entering gesture mode.");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            GTP_INFO("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send gesture cmd failed.");
    return ret;
}

/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
    u8 status_buf[3] = {0x80, 0x44};
#endif

    GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // GT9XXF: host interact with ic
        ret = gtp_i2c_read(ts->client, status_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("failed to get backup-reference status");
        }

        if (status_buf[2] & 0x80)
        {
            ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
            if (FAIL == ret)
            {
                GTP_ERROR("failed to store bak_ref");
            }
        }
    }
#endif

    GTP_GPIO_OUTPUT(gtp_int_gpio, 0);
    msleep(5);

    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_INFO("GTP enter sleep!");

            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

#if GTP_COMPATIBLE_MODE
#if GTP_GESTURE_WAKEUP
    if(!doze_switch_flag)
    {
#endif
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            u8 opr_buf[3] = {0x41, 0x80};

            GTP_GPIO_OUTPUT(gtp_int_gpio, 1);
            msleep(5);

            for (retry = 0; retry < 10; ++retry)
            {
                // hold ss51 & dsp
                opr_buf[2] = 0x0C;
                ret = gtp_i2c_write(ts->client, opr_buf, 3);
                if (FAIL == ret)
                {
                    GTP_ERROR("failed to hold ss51 & dsp!");
                    continue;
                }
                opr_buf[2] = 0x00;
                ret = gtp_i2c_read(ts->client, opr_buf, 3);
                if (FAIL == ret)
                {
                    GTP_ERROR("failed to get ss51 & dsp status!");
                    continue;
                }
                if (0x0C != opr_buf[2])
                {
                    GTP_DEBUG("ss51 & dsp not been hold, %d", retry+1);
                    continue;
                }
                GTP_DEBUG("ss51 & dsp confirmed hold");

                ret = gtp_fw_startup(ts->client);
                if (FAIL == ret)
                {
                    GTP_ERROR("failed to startup GT9XXF, process recovery");
                    gtp_esd_recovery(ts->client);
                }
                break;
            }
            if (retry >= 10)
            {
                GTP_ERROR("failed to wakeup, processing esd recovery");
                gtp_esd_recovery(ts->client);
            }
            else
            {
                GTP_INFO("GT9XXF gtp wakeup success");
            }
            return ret;
        }
#if GTP_GESTURE_WAKEUP
    }
#endif
#endif

#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_912_reset_guitar(ts->client, 20);

        GTP_INFO("GTP wakeup sleep.");
        return 1;
    }
#else
    while(retry++ < 10)
    {
    #if GTP_GESTURE_WAKEUP
        if(doze_switch_flag)
        {
            if (DOZE_WAKEUP != doze_status)
            {
                GTP_INFO("Powerkey wakeup.");
            }
            else
            {
                GTP_INFO("Gesture wakeup.");
            }
            doze_status = DOZE_DISABLED;
            gtp_912_irq_disable(ts);
            gtp_912_reset_guitar(ts->client, 10);
            ret = gup_reload_fw_dsp(NULL, GTP_FL_READ_REPAIR);
            gtp_912_irq_enable(ts);
            if (ret == FAIL) {
                GTP_ERROR("[%s]Reload dsp code failed.", __func__);
                continue;
            }
            ret = gtp_fw_startup(ts->client);
            if (ret == FAIL) {
                GTP_ERROR("[%s]Startup fw failed.", __func__);
                gtp_recovery_reset(ts->client);
            }
            return 0;
        }
        else
        {
            GTP_GPIO_OUTPUT(gtp_int_gpio, 1);
            msleep(5);
        }
    #else
        GTP_GPIO_OUTPUT(gtp_int_gpio, 1);
        msleep(5);
    #endif

        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            GTP_INFO("GTP wakeup sleep.");

        #if (!GTP_GESTURE_WAKEUP)
            {
                gtp_int_sync(25);
            #if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
            #endif
            }
        #endif
        #if (GTP_GESTURE_WAKEUP)
            if(!doze_switch_flag)
            {
                gtp_int_sync(25);
            #if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
            #endif
            }
        #endif

            return ret;
        }
        gtp_912_reset_guitar(ts->client, 20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    u8 sensor_id = 0;
	u8 drv_cfg_version;
	u8 flash_cfg_version;

/* if defined CONFIG_OF, parse config data from dtsi
 *  else parse config data form header file.
 */
//#ifndef	CONFIG_OF
#if 1
    u8 cfg_info_group0[] = CTP_CFG_GROUP0;
    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;

    u8 *send_cfg_buf[] = {cfg_info_group0,cfg_info_group1,
						cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group0),
						  CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5)};

    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);
#endif

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type) {
        ts->fw_error = 0;
    }
	else
#endif
    {	/* check firmware */
        ret = gtp_912_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
        if (SUCCESS == ret)
		{
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                GTP_ERROR("Firmware error, no config sent!");
                return -1;
            }
        }
    }

	/* read sensor id */
#if GTP_COMPATIBLE_MODE
    msleep(50);
#endif
    ret = gtp_912_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
    if (SUCCESS == ret)
    {
        if (sensor_id >= 0x06)
        {
            GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
            ts->pnl_init_error = 1;
            return -1;
        }
    }
    else
    {
        GTP_ERROR("Failed to get sensor_id, No config sent!");
        ts->pnl_init_error = 1;
        return -1;
    }
    GTP_INFO("Sensor_ID: %d", sensor_id);

	/* parse config data*/
//#ifdef CONFIG_OF
#if 0
    GTP_DEBUG("Get config data from device tree.");
	ret = gtp_parse_dt_cfg(&ts->client->dev, &gt912_config[GTP_ADDR_LENGTH], &ts->gtp_cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse config data form device tree.");
		ts->pnl_init_error = 1;
		return -1;
	}
#else
	GTP_DEBUG("Get config data from header file.");
    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
        (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
        (!cfg_info_len[5]))
    {
        sensor_id = 0;
    }
	ts->gtp_cfg_len = cfg_info_len[sensor_id];
	memset(&gt912_config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&gt912_config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);
#endif

    GTP_INFO("Config group%d used,length: %d", sensor_id, ts->gtp_cfg_len);

    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }

#if GTP_COMPATIBLE_MODE
	if (ts->chip_type != CHIP_TYPE_GT9F)
#endif
	{
	    ret = gtp_912_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
	    if (ret == SUCCESS) {
	        GTP_DEBUG("Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X",
	                    gt912_config[GTP_ADDR_LENGTH], gt912_config[GTP_ADDR_LENGTH], opr_buf[0], opr_buf[0]);

			flash_cfg_version = opr_buf[0];
			drv_cfg_version = gt912_config[GTP_ADDR_LENGTH];
			if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
				gt912_config[GTP_ADDR_LENGTH] = 0x00;
			}
	    } else {
	        GTP_ERROR("Failed to get ic config version!No config sent!");
	        return -1;
	    }
	}

#if GTP_CUSTOM_CFG
    gt912_config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    gt912_config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    gt912_config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    gt912_config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        gt912_config[TRIGGER_LOC] &= 0xfe;
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        gt912_config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += gt912_config[i];
    }
    gt912_config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }

#endif // GTP_DRIVER_SEND_CFG

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (gt912_config[RESOLUTION_LOC + 1] << 8) + gt912_config[RESOLUTION_LOC];
        ts->abs_y_max = (gt912_config[RESOLUTION_LOC + 3] << 8) + gt912_config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (gt912_config[TRIGGER_LOC]) & 0x03;
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 sensor_num = 0;
        u8 driver_num = 0;
        u8 have_key = 0;

        have_key = (gt912_config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);

        if (1 == ts->is_950)
        {
            driver_num = gt912_config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = gt912_config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
            if (have_key)
            {
                driver_num--;
            }
            ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
        }
        else
        {
            driver_num = (gt912_config[CFG_LOC_DRVA_NUM] & 0x1F) + (gt912_config[CFG_LOC_DRVB_NUM]&0x1F);
            if (have_key)
            {
                driver_num--;
            }
            sensor_num = (gt912_config[CFG_LOC_SENS_NUM] & 0x0F) + ((gt912_config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
            ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
        }

        GTP_INFO("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
           driver_num, sensor_num, have_key, ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
        return 0;
    }
    else
#endif
    {
#if GTP_DRIVER_SEND_CFG
        ret = gtp_912_send_cfg(ts->client);
        if (ret < 0)
        {
            GTP_ERROR("Send config error.");
        }
#if GTP_COMPATIBLE_MODE
	if (ts->chip_type != CHIP_TYPE_GT9F)
#endif
	{
		if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
			check_sum = 0;
	        gt912_config[GTP_ADDR_LENGTH] = drv_cfg_version;
			for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
				check_sum += gt912_config[i];
			}
			gt912_config[ts->gtp_cfg_len] = (~check_sum) + 1;
		}
	}

#endif
        GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    }

    msleep(10);
    return 0;

}

static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0x80, 0x47};
    int i;

    if (*ppos)
    {
        return 0;
    }
    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", gt912_config[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    gtp_i2c_read(gtp_912_i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i+2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}
static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;

    GTP_DEBUG("write count %d\n", count);

    if (count > GTP_CONFIG_MAX_LENGTH)
    {
        GTP_ERROR("size not match [%d:%d]\n", GTP_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&gt912_config[2], buffer, count))
    {
        GTP_ERROR("copy from user fail\n");
        return -EFAULT;
    }

    ret = gtp_912_send_cfg(gtp_912_i2c_connect_client);

    if (ret < 0)
    {
        GTP_ERROR("send config failed.");
    }

    return count;
}
/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
 s32 gtp_912_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    if (buf[5] == 0x00)
    {
        GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
    s32 ret = 0;

    GTP_DEBUG_FUNC();
    ret = GTP_GPIO_REQUEST(gtp_int_gpio, "GTP INT IRQ");
    if (ret < 0)
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32)gtp_int_gpio, ret);
        ret = -ENODEV;
    }
    else
    {
        GTP_GPIO_AS_INT(gtp_int_gpio);
        ts->client->irq = gpio_to_irq(gtp_int_gpio);
    }

    ret = GTP_GPIO_REQUEST(gtp_rst_gpio, "GTP RST PORT");
    if (ret < 0)
    {
        GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",(s32)gtp_rst_gpio,ret);
        ret = -ENODEV;
    }

    GTP_GPIO_AS_INPUT(gtp_rst_gpio);

    gtp_912_reset_guitar(ts->client, 20);

    if(ret < 0)
    {
        GTP_GPIO_FREE(gtp_rst_gpio);
        GTP_GPIO_FREE(gtp_int_gpio);
    }

    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static void gtp_release_io_port(struct goodix_ts_data *ts)
{

if (gpio_is_valid(gtp_rst_gpio))
		gpio_free(gtp_rst_gpio);
	if (gpio_is_valid(gtp_int_gpio))
		gpio_free(gtp_int_gpio);
   if (ts)
    {
        if (ts->use_irq)
        {
           free_irq(ts->client->irq, ts);
        }

    }
    return ;
}


/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    const u8 irq_table[] = GTP_IRQ_TAB;

    GTP_DEBUG_FUNC();
    GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

    ret  = request_irq(ts->client->irq,
                       goodix_ts_irq_handler,
                       irq_table[ts->int_trigger_type],
                       ts->client->name,
                       ts);
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        GTP_GPIO_AS_INPUT(gtp_int_gpio);
        GTP_GPIO_FREE(gtp_int_gpio);

        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else
    {
        gtp_912_irq_disable(ts);
        ts->use_irq = 1;
        return 0;
    }
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif

    GTP_DEBUG_FUNC();

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GTP_ICS_SLOT_REPORT
    input_mt_init_slots(ts->input_dev, 16,0);     // in case of "out of memory"
#else
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#if GTP_HAVE_TOUCH_KEY
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);
    }
#endif

#if (GTP_GESTURE_WAKEUP || GTP_PALM_SUSPEND)
    input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_CHANGE_X2Y
    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = goodix_input_phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;

    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }

#if GTP_WITH_PEN
    gtp_pen_init(ts);
#endif

    return 0;
}

//************** For GT9XXF Start *************//
#if GTP_COMPATIBLE_MODE

s32 gtp_fw_startup(struct i2c_client *client)
{
    u8 opr_buf[4];
    s32 ret = 0;

    //init sw WDT
	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }

    //release SS51 & DSP
    opr_buf[0] = 0x00;
    ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    //int sync
    gtp_int_sync(25);

    //check fw run status
    ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    if(0xAA == opr_buf[0])
    {
        GTP_ERROR("IC works abnormally,startup failed.");
        return FAIL;
    }
    else
    {
        GTP_INFO("IC works normally, Startup success.");
        opr_buf[0] = 0xAA;
        i2c_write_bytes(client, 0x8041, opr_buf, 1);
        return SUCCESS;
    }
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
    s32 retry = 0;
    s32 ret = 0;
    struct goodix_ts_data *ts;

    ts = i2c_get_clientdata(client);

    gtp_912_irq_disable(ts);

    GTP_INFO("GT9XXF esd recovery mode");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
        if (FAIL == ret)
        {
            GTP_ERROR("esd recovery failed %d", retry+1);
            continue;
        }
        ret = gtp_fw_startup(ts->client);
        if (FAIL == ret)
        {
            GTP_ERROR("GT9XXF start up failed %d", retry+1);
            continue;
        }
        break;
    }
    gtp_912_irq_enable(ts);

    if (retry >= 5)
    {
        GTP_ERROR("failed to esd recovery");
        return FAIL;
    }

    GTP_INFO("Esd recovery successful");
    return SUCCESS;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
    gtp_912_esd_switch(client, SWITCH_OFF);
#endif
    GTP_DEBUG_FUNC();

    gtp_esd_recovery(client);

#if GTP_ESD_PROTECT
    gtp_912_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
    s32 ret = 0;
    s32 i = 0;
    s32 j = 0;
    u16 ref_sum = 0;
    u16 learn_cnt = 0;
    u16 chksum = 0;
    s32 ref_seg_len = 0;
    s32 ref_grps = 0;
    struct file *ref_filp = NULL;
    u8 *p_bak_ref;

    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->ref_chk_fs_times++;
        GTP_DEBUG("Ref check /data times/MAX_TIMES: %d / %d", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
            GTP_INFO("/data not mounted.");
            return FAIL;
        }
        GTP_INFO("check /data mount timeout...");
    }
    else
    {
        GTP_INFO("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }

    p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);

    if (NULL == p_bak_ref)
    {
        GTP_ERROR("Allocate memory for p_bak_ref failed!");
        return FAIL;
    }

    if (ts->is_950)
    {
        ref_seg_len = ts->bak_ref_len / 6;
        ref_grps = 6;
    }
    else
    {
        ref_seg_len = ts->bak_ref_len;
        ref_grps = 1;
    }
    ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(ref_filp))
    {
        GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
        if (GTP_BAK_REF_SEND == mode)
        {
            goto bak_ref_default;
        }
        else
        {
            goto bak_ref_exit;
        }
    }

    switch (mode)
    {
    case GTP_BAK_REF_SEND:
        GTP_INFO("Send backup-reference");
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ret = ref_filp->f_op->read(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        if (ret < 0)
        {
            GTP_ERROR("failed to read bak_ref info from file, sending defualt bak_ref");
            goto bak_ref_default;
        }
        for (j = 0; j < ref_grps; ++j)
        {
            ref_sum = 0;
            for (i = 0; i < (ref_seg_len); i += 2)
            {
                ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i+1 + j * ref_seg_len];
            }
            learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len -4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -3]);
            chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len -2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -1]);
            GTP_DEBUG("learn count = %d", learn_cnt);
            GTP_DEBUG("chksum = %d", chksum);
            GTP_DEBUG("ref_sum = 0x%04X", ref_sum & 0xFFFF);
            // Sum(1~ref_seg_len) == 1
            if (1 != ref_sum)
            {
                GTP_INFO("wrong chksum for bak_ref, reset to 0x00 bak_ref");
                memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
            }
            else
            {
                if (j == (ref_grps - 1))
                {
                    GTP_INFO("backup-reference data in %s used", GTP_BAK_REF_PATH);
                }
            }
        }
        ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (FAIL == ret)
        {
            GTP_ERROR("failed to send bak_ref because of iic comm error");
            goto bak_ref_exit;
        }
        break;

    case GTP_BAK_REF_STORE:
        GTP_INFO("Store backup-reference");
        ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (ret < 0)
        {
            GTP_ERROR("failed to read bak_ref info, sending default back-reference");
            goto bak_ref_default;
        }
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        break;

    default:
        GTP_ERROR("invalid backup-reference request");
        break;
    }
    ret = SUCCESS;
    goto bak_ref_exit;

bak_ref_default:

    for (j = 0; j < ref_grps; ++j)
    {
        memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
        p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1
    }
    ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
    if (!IS_ERR(ref_filp))
    {
        GTP_INFO("write backup-reference data into %s", GTP_BAK_REF_PATH);
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
    }
    if (ret == FAIL)
    {
        GTP_ERROR("failed to load the default backup reference");
    }

bak_ref_exit:

    if (p_bak_ref)
    {
        kfree(p_bak_ref);
    }
    if (ref_filp && !IS_ERR(ref_filp))
    {
        filp_close(ref_filp, NULL);
    }
    return ret;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
    u8 chksum = 0;
    u8 main_clock = p_main_clk[0];
    s32 i = 0;

    if (main_clock < 50 || main_clock > 120)
    {
        return FAIL;
    }

    for (i = 0; i < 5; ++i)
    {
        if (main_clock != p_main_clk[i])
        {
            return FAIL;
        }
        chksum += p_main_clk[i];
    }
    chksum += p_main_clk[5];
    if ( (chksum) == 0)
    {
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
    s32 ret = 0;
    s32 i = 0;
    s32 clk_chksum = 0;
    struct file *clk_filp = NULL;
    u8 p_main_clk[6] = {0};

    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->clk_chk_fs_times++;
        GTP_DEBUG("Clock check /data times/MAX_TIMES: %d / %d", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
            GTP_INFO("/data not mounted.");
            return FAIL;
        }
        GTP_INFO("Check /data mount timeout!");
    }
    else
    {
        GTP_INFO("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }

    clk_filp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(clk_filp))
    {
        GTP_ERROR("%s is unavailable, calculate main clock", GTP_MAIN_CLK_PATH);
    }
    else
    {
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);

        ret = gtp_verify_main_clk(p_main_clk);
        if (FAIL == ret)
        {
            // recalculate main clock & rewrite main clock data to file
            GTP_ERROR("main clock data in %s is wrong, recalculate main clock", GTP_MAIN_CLK_PATH);
        }
        else
        {
            GTP_INFO("main clock data in %s used, main clock freq: %d", GTP_MAIN_CLK_PATH, p_main_clk[0]);
            filp_close(clk_filp, NULL);
            goto update_main_clk;
        }
    }

#if GTP_ESD_PROTECT
    gtp_912_esd_switch(ts->client, SWITCH_OFF);
#endif
    ret = gup_clk_calibration();
    gtp_esd_recovery(ts->client);

#if GTP_ESD_PROTECT
    gtp_912_esd_switch(ts->client, SWITCH_ON);
#endif

    GTP_INFO("calibrate main clock: %d", ret);
    if (ret < 50 || ret > 120)
    {
        GTP_ERROR("wrong main clock: %d", ret);
        goto exit_main_clk;
    }

    // Sum{0x8020~0x8025} = 0
    for (i = 0; i < 5; ++i)
    {
        p_main_clk[i] = ret;
        clk_chksum += p_main_clk[i];
    }
    p_main_clk[5] = 0 - clk_chksum;

    if (!IS_ERR(clk_filp))
    {
        GTP_DEBUG("write main clock data into %s", GTP_MAIN_CLK_PATH);
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
        filp_close(clk_filp, NULL);
    }

update_main_clk:
    ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
    if (FAIL == ret)
    {
        GTP_ERROR("update main clock failed!");
        return FAIL;
    }
    return SUCCESS;

exit_main_clk:
    if (!IS_ERR(clk_filp))
    {
        filp_close(clk_filp, NULL);
    }
    return FAIL;
}


s32 gtp_gt9xxf_init(struct i2c_client *client)
{
    s32 ret = 0;
    GTP_DEBUG_FUNC();
    ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
    if (FAIL == ret)
    {
        return FAIL;
    }

    ret = gtp_fw_startup(client);
    if (FAIL == ret)
    {
        return FAIL;
    }
    return SUCCESS;
}

static void gtp_get_chip_type(struct goodix_ts_data *ts)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;

    msleep(10);

    ret = gtp_912_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);

    if (FAIL == ret)
    {
        GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
        ts->chip_type = CHIP_TYPE_GT9;
        return;
    }

    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        ts->chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        ts->chip_type = CHIP_TYPE_GT9F;
    }
    GTP_INFO("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}
#endif     //For GT9XXF End//

/*
 * Devices Tree support,
*/
#ifdef CONFIG_OF
/**
 * gtp_parse_dt - parse platform infomation form devices tree.
 */
static void gtp_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;

	gtp_int_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	gtp_rst_gpio = of_get_named_gpio(np, "goodix,rst-gpio", 0);
}

/**
 * gtp_parse_dt_cfg - parse config data from devices tree.
 * @dev: device that this driver attached.
 * @cfg: pointer of the config array.
 * @cfg_len: pointer of the config length.
 * @sid: sensor id.
 * Return: 0-succeed, -1-faileds
 */
int gtp_parse_dt_cfg(struct device *dev, u8 *cfg, int *cfg_len, u8 sid)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	char cfg_name[18];

	snprintf(cfg_name, sizeof(cfg_name), "goodix,cfg-group%d", sid);
	prop = of_find_property(np, cfg_name, cfg_len);
	if (!prop || !prop->value || *cfg_len == 0 || *cfg_len > GTP_CONFIG_MAX_LENGTH) {
		return -1;/* failed */
	} else {
		memcpy(cfg, prop->value, *cfg_len);
		return 0;
	}
}

/**
 * gtp_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
static int gtp_power_switch(struct i2c_client *client, int on)
{
	static struct regulator *vdd_ana;
	static struct regulator *vcc_i2c;
	int ret;

	if (!vdd_ana) {
		vdd_ana = regulator_get(&client->dev, "vdd_ana");
		if (IS_ERR(vdd_ana)) {
			GTP_ERROR("regulator get of vdd_ana failed");
			ret = PTR_ERR(vdd_ana);
			vdd_ana = NULL;
			return ret;
		}
	}

	if (!vcc_i2c) {
		vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(vcc_i2c)) {
			GTP_ERROR("regulator get of vcc_i2c failed");
			ret = PTR_ERR(vcc_i2c);
			vcc_i2c = NULL;
			goto ERR_GET_VCC;
		}
	}

	if (on) {
		GTP_DEBUG("GTP power on.");
		ret = regulator_enable(vdd_ana);
		udelay(2);
		ret = regulator_enable(vcc_i2c);
	} else {
		GTP_DEBUG("GTP power off.");
		ret = regulator_disable(vcc_i2c);
		udelay(2);
		ret = regulator_disable(vdd_ana);
	}
	return ret;

ERR_GET_VCC:
	regulator_put(vdd_ana);
	return ret;
}
#endif

/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
struct gtp_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

static struct gtp_pinctrl_info gt9xx_pctrl;
static int gtp_pinctrl_init(struct device *dev)
{
	gt9xx_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	gt9xx_pctrl.gpio_state_active = pinctrl_lookup_state(
					       gt9xx_pctrl.pinctrl,
					       GOODIX_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	gt9xx_pctrl.gpio_state_suspend = pinctrl_lookup_state(
						gt9xx_pctrl.pinctrl,
						GOODIX_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(gt9xx_pctrl.gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/


/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes.
        0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct goodix_ts_data *ts;
    u16 version_info;

    GTP_DEBUG_FUNC();

    //do NOT remove these logs
    GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
    client->addr=0x5d;
    GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

    gtp_912_i2c_connect_client = client;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }

#ifdef CONFIG_OF	/* device tree support */
    if (client->dev.of_node) {
		gtp_parse_dt(&client->dev);
    }
    ret = gtp_power_switch(client, 1);
	if (ret) {
		GTP_ERROR("GTP power on failed.");
		return -EINVAL;
	}
#else			/* use gpio defined in gt9xx.h */
	gtp_rst_gpio = GTP_RST_PORT;
	gtp_int_gpio = GTP_INT_PORT;
#endif

    INIT_WORK(&ts->work, goodix_ts_work_func);
    ts->client = client;
    spin_lock_init(&ts->irq_lock);          // 2.6.39 later
    // ts->irq_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
#if GTP_ESD_PROTECT
    ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
    GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
    spin_lock_init(&ts->esd_lock);
    // ts->esd_lock = SPIN_LOCK_UNLOCKED;
#endif
    i2c_set_clientdata(client, ts);
    ts->gtp_rawdiff_mode = 0;
    ret = gtp_request_io_port(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.");
        kfree(ts);
        return ret;
    }
	/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
		gtp_pinctrl_init(&ts->client->dev);
		ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
				gt9xx_pctrl.gpio_state_active);
		if (ret)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

     ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
    }
#if GTP_COMPATIBLE_MODE
    gtp_get_chip_type(ts);
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {    GTP_DEBUG_FUNC();
        ret = gtp_gt9xxf_init(ts->client);
        if (FAIL == ret)
        {
            GTP_INFO("Failed to init GT9XXF.");
        }
    }
    else
    {
     GTP_ERROR("not GT912 !");
     goto gpio_free;
    }
#endif
    ret = gtp_912_read_version(client, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }

    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }

    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        GTP_INFO("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }

#if GTP_ESD_PROTECT
    gtp_912_esd_switch(client, SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(ts);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif

    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
    }

    ret = gtp_request_irq(ts);
    if (ret < 0)
    {
        GTP_INFO("GTP works in polling mode.");
    }
    else
    {
        GTP_INFO("GTP works in interrupt mode.");
    }

    if (ts->use_irq)
    {
        gtp_912_irq_enable(ts);
#if GTP_GESTURE_WAKEUP
	enable_irq_wake(client->irq);
#endif
    }
	firm_ver_attr_create(); //[FEATURE]-Add-begin by TCTNB.TianHongwei 2014/5/19 show fw version for app.
//Add begin by TCTNB-QW  FR978057  2015/04/23
//[NEW FEATURE] add for gesture switch attr file
#if (GTP_GESTURE_WAKEUP || GTP_PALM_SUSPEND)
    gesture_switch_attr_create();
#endif
#if CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	ret= virtual_psensor_input_register(client);
	if(ret< 0)
	{
		pr_err("[***%s: failed\n", __func__);
		return ret;
	}
#endif
//Add end by TCTNB-QW  FR978057  2015/04/23

	/* register suspend and resume fucntion*/
	gtp_register_powermanger(ts);
#if GTP_CREATE_WR_NODE
    gt912_init_wr_node(client);
#endif
    return 0;
gpio_free:
    gtp_release_io_port(ts);
    return -EIO;

}

//[FEATURE]-Add-Begin by TCTNB.QW 2015/04/28 	FR990180	show fw version for app.

static DEVICE_ATTR(firm_ver, 0444, firm_ver_show, NULL);
s32 gtp_read_version(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(ts->client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (buf[5] == 0x00)
    {
        GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }

     ts->ic_vid = (buf[7] << 8) | buf[6];
     memcpy(ts->ic_pid,&buf[2],4);
     ts->ic_pid[4] = 0 ;

    return ret;
}

u8 gup_get_fw_version(u16 *fw_info,u8 *module)
{
#if GTP_AUTO_UPDATE
	st_fw_head fw_head;
    #if GTP_HEADER_FW_UPDATE
	memcpy(&fw_head, &gtp_default_FW[0], FW_HEAD_LENGTH);
	*fw_info =(u16)(fw_head.vid>>8) + (u16)(fw_head.vid<<8);
	memcpy(module,fw_head.pid,4);
	module[4]=0;
    #endif
	return 1;
#endif
	return 0;
}

static struct class *firm_ver_class;
static struct device *firm_ver_dev;
static ssize_t firm_ver_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{

   int ret;
   u16 fw_info;
   u8 fw_pid[5];

   struct goodix_ts_data *ts = i2c_get_clientdata(gtp_912_i2c_connect_client);

   if (!ts->gtp_is_suspend){
	   ret = gtp_read_version(ts);
	    if (ret < 0) {
	        pr_err( "GTP read ic version  failed.\n");
	    }
   }
   ret = gup_get_fw_version(&fw_info,fw_pid);
   if(!ret)
   {
	fw_info = ts->ic_vid;
	memcpy(fw_pid,ts->ic_pid,5);
	pr_err( "GTP read fw version  failed.\n");
   }

    return sprintf ( buf, "fwversion:0x%x \nfwhostversion:0x%x \nfwmoduleid:%s  \n",ts->ic_vid,fw_info,fw_pid);//%x-->%d,for hardware demand
}

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

//[NEW FEATURE] add for gesture switch attr file 2015/07/01
#if (GTP_GESTURE_WAKEUP || GTP_PALM_SUSPEND)
static void gesture_switch_attr_create(void)
{
   gesture_switch_class = class_create(THIS_MODULE, "gesture_switch");
   if (IS_ERR(gesture_switch_class))
        pr_err("Failed to create class(gesture_switch_class)!\n");
    gesture_switch_dev = device_create(gesture_switch_class,
                                     NULL, 0, NULL, "device");
   if (IS_ERR(gesture_switch_dev))
        pr_err("Failed to create device(gesture_switch_dev)!\n");
#if GTP_GESTURE_WAKEUP
    if(device_create_file(gesture_switch_dev,&dev_attr_doze_switch) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_doze_switch.attr.name);
#endif
#if GTP_PALM_SUSPEND
    if(device_create_file(gesture_switch_dev,&dev_attr_palm_switch) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_doze_switch.attr.name);
#endif
}
#endif
//[FEATURE]-Add-end by TCTNB.QW 2015/04/28 	FR990180	show fw version for app.

/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    GTP_DEBUG_FUNC();
    printk(KERN_CRIT"GT912 goodix_ts_remove \n");
	gtp_unregister_powermanger(ts);

#if GTP_CREATE_WR_NODE
    gt912_uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    if (ts)
    {
        if (ts->use_irq)
        {
            GTP_GPIO_AS_INPUT(gtp_int_gpio);
            GTP_GPIO_FREE(gtp_int_gpio);
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }

    GTP_INFO("GTP driver removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    kfree(ts);

    return 0;
}


/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int goodix_ts_suspend(struct goodix_ts_data *ts)
{
    s8 ret = -1;

    GTP_DEBUG_FUNC();
    if (ts->enter_update) {
    return 0;
    }
    GTP_INFO("System suspend.");

    ts->gtp_is_suspend = 1;
#if GTP_ESD_PROTECT
    gtp_912_esd_switch(ts->client, SWITCH_OFF);
#endif

/*[BUGFIX]Add Begin by TCTNB-QW 2015-04-23,FR-978057*/
#if CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(vps->vps_enabled)
		return 0;
#endif
/*[BUGFIX]Add End by TCTNB-QW 2015-04-23,FR-978057*/

/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
	ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
			gt9xx_pctrl.gpio_state_suspend);
	if (ret)
		pr_err("%s:%d cannot set pin to suspend state",
			__func__, __LINE__);
/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

#if GTP_GESTURE_WAKEUP
    if(doze_switch_flag)
        ret = gtp_enter_doze(ts);
    else{
        if (ts->use_irq)
        {
            gtp_912_irq_disable(ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
        ret = gtp_enter_sleep(ts);
    }
#else
    if (ts->use_irq)
    {
        gtp_912_irq_disable(ts);
    }
    else
    {
        hrtimer_cancel(&ts->timer);
    }
    ret = gtp_enter_sleep(ts);
#endif
#ifdef CONFIG_SENSORS_HALL
	hall->tp_is_suspend = 1;
#endif
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping
    //  delay 48 + 10ms to ensure reliability
    msleep(58);
	return 0;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_resume(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    GTP_DEBUG_FUNC();
   // if (ts->enter_update){  //refer to pixi3-8 VDF, TP can't work after reboot it will affect the usb port identify -yuan jianeng
    if (ts->enter_update || ts->gtp_is_suspend == 0){
    return;
    }
    GTP_INFO("System resume.");

	/*[BUGFIX]Add Begin by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/
		ret = pinctrl_select_state(gt9xx_pctrl.pinctrl,
				gt9xx_pctrl.gpio_state_active);
		if (ret)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	/*[BUGFIX]Add End by TCTSZ-LZ 2014-5-5,PR-667466. TP INT pull-up enable.*/

    ret = gtp_wakeup_sleep(ts);
#if GTP_GESTURE_WAKEUP
    if(doze_switch_flag){
        doze_status = DOZE_DISABLED;
    }
#endif

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }
#if (GTP_COMPATIBLE_MODE)
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // do nothing
    }
    else
#endif
    {
        gtp_912_send_cfg(ts->client);
    }

    if (ts->use_irq)
    {
        gtp_912_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;
#ifdef CONFIG_SENSORS_HALL
	hall->tp_is_suspend = 0;
#endif
#if GTP_ESD_PROTECT
    gtp_912_esd_switch(ts->client, SWITCH_ON);
#endif

}


#if   defined(CONFIG_FB)
static void fb_notify_resume_work(struct work_struct *work)
{
	   struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, fb_notify_work);
       goodix_ts_resume(ts);
}

/* frame buffer notifier block control the suspend/resume procedure */
static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	struct goodix_ts_data *ts = container_of(noti, struct goodix_ts_data, notifier);
	int *blank;

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK && ts) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK) {
			GTP_DEBUG("Resume by fb notifier.");
			schedule_work(&ts->fb_notify_work);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			GTP_DEBUG("Suspend by fb notifier.");
			flush_work(&ts->fb_notify_work);
			goodix_ts_suspend(ts);
		}
	}

	return 0;
}
#elif defined(CONFIG_PM)
/* bus control the suspend/resume procedure */
static int gtp_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts) {
		GTP_DEBUG("Suspend by i2c pm.");
		goodix_ts_suspend(ts);
	}

	return 0;
}
static int gtp_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts) {
		GTP_DEBUG("Resume by i2c pm.");
		goodix_ts_resume(ts);
	}

	return 0;
}

static struct dev_pm_ops gtp_pm_ops = {
	.suspend = gtp_pm_suspend,
	.resume  = gtp_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void gtp_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts = container_of(h, struct goodix_ts_data, early_suspend);

	if (ts) {
		GTP_DEBUG("Suspend by earlysuspend module.");
		goodix_ts_suspend(ts);
	}
}
static void gtp_early_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts = container_of(h, struct goodix_ts_data, early_suspend);

	if (ts) {
		GTP_DEBUG("Resume by earlysuspend module.");
		goodix_ts_resume(ts);
	}
}
#endif

static int gtp_register_powermanger(struct goodix_ts_data *ts)
{
#if   defined(CONFIG_FB)
	INIT_WORK(&ts->fb_notify_work, fb_notify_resume_work);
	ts->notifier.notifier_call = gtp_fb_notifier_callback;
	fb_register_client(&ts->notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;
}

static int gtp_unregister_powermanger(struct goodix_ts_data *ts)
{
#if   defined(CONFIG_FB)
		fb_unregister_client(&ts->notifier);

#elif defined(CONFIG_HAS_EARLYSUSPEND)
		unregister_early_suspend(&ts->early_suspend);
#endif
return 0;
}

/* end */

#if GTP_ESD_PROTECT
s32 gtp_912_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

s32 gtp_912_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_912_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_ts_data *ts;

    ts = i2c_get_clientdata(client);
    spin_lock(&ts->esd_lock);

    if (SWITCH_ON == on)     // switch on esd
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
            GTP_INFO("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        } else {
					spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            GTP_INFO("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
    GTP_DEBUG("[Esd]Init external watchdog");
    return gtp_912_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    u8 esd_buf[5] = {0x80, 0x40};
    GTP_DEBUG_FUNC();
    ts = i2c_get_clientdata(gtp_912_i2c_connect_client);

    if (ts->gtp_is_suspend || ts->enter_update)
    {
        GTP_INFO("Esd suspended!");
        return;
    }
    for (i = 0; i < 3; i++)
    {
        ret = gtp_912_i2c_read_no_rst(ts->client, esd_buf, 4);

        GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        {
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};

                gtp_912_i2c_read_no_rst(ts->client, chk_buf, 4);
                GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);
                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = 3;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA;
                gtp_912_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            if (ts->rqst_processing)
            {
                GTP_INFO("Request processing, no esd recovery");
            }
            else
            {
                GTP_ERROR("IC working abnormally! Process esd recovery.");
                esd_buf[0] = 0x42;
                esd_buf[1] = 0x26;
                esd_buf[2] = 0x01;
                esd_buf[3] = 0x01;
                esd_buf[4] = 0x01;
                gtp_912_i2c_write_no_rst(ts->client, esd_buf, 5);
                msleep(50);
		#ifdef CONFIG_OF
				gtp_power_switch(ts->client, 0);
				msleep(20);
				gtp_power_switch(ts->client, 1);
				msleep(20);
		#endif
                gtp_esd_recovery(ts->client);
            }
        }
        else
    #endif
        {
            GTP_ERROR("IC working abnormally! Process reset guitar.");
            esd_buf[0] = 0x42;
            esd_buf[1] = 0x26;
            esd_buf[2] = 0x01;
            esd_buf[3] = 0x01;
            esd_buf[4] = 0x01;
            gtp_912_i2c_write_no_rst(ts->client, esd_buf, 5);
            msleep(50);
		#ifdef CONFIG_OF
			gtp_power_switch(ts->client, 0);
			msleep(20);
			gtp_power_switch(ts->client, 1);
			msleep(20);
		#endif
            gtp_912_reset_guitar(ts->client, 50);
            msleep(50);
            gtp_912_send_cfg(ts->client);
        }
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        GTP_INFO("Esd suspended!");
    }
    return;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id goodix_match_table[] = {
		{.compatible = "goodix,gt912",},
		{ },
};
#endif

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME_GT912, 0 },
    { }
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME_GT912,
        .owner    = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = goodix_match_table,
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		.pm		  = &gtp_pm_ops,
#endif
    },
};

/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int  goodix_ts_init(void)
{
    s32 ret;

    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver installing...");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
#if GTP_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
    ret = i2c_add_driver(&goodix_ts_driver);
    return ret;
}

/*******************************************************
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
