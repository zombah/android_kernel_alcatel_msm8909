/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/sensors.h>
#include <linux/input/ft5x06_ts.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define FT_PROC_DEBUG
#define FT_ITO_TEST
#define FOCALTECH_PWRON_UPGRADE

#if defined(FT_PROC_DEBUG)
#include <linux/proc_fs.h>
#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00
#endif

#if defined(FT_ITO_TEST)
#include <asm/uaccess.h>
#include "ft_rawdata.h"
static struct i2c_client  *ft_g_client;
static struct mutex g_rawdata_mutex;
#endif

#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_FW_VENDOR_ID	0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* psensor register address*/
#define FT_REG_PSENSOR_ENABLE	0xB0
#define FT_REG_PSENSOR_STATUS	0x01

/* psensor register bits*/
#define FT_PSENSOR_ENABLE_MASK	0x01
#define FT_PSENSOR_STATUS_NEAR	0xC0
#define FT_PSENSOR_STATUS_FAR	0xE0
#define FT_PSENSOR_FAR_TO_NEAR	0
#define FT_PSENSOR_NEAR_TO_FAR	1
#define FT_PSENSOR_ORIGINAL_STATE_FAR	1
#define FT_PSENSOR_WAKEUP_TIMEOUT	500

/* gesture register address*/
#define FT_REG_GESTURE_ENABLE	0xD0
#define FT_REG_GESTURE_OUTPUT	0xD3

/* gesture register bits*/
#define FT_GESTURE_DOUBLECLICK_COORD_X		100
#define FT_GESTURE_DOUBLECLICK_COORD_Y		100
#define FT_GESTURE_WAKEUP_TIMEOUT		500
#define FT_GESTURE_DEFAULT_TRACKING_ID		0x0A
#define FT_GESTURE_DOUBLECLICK_ID		0x24
#define FT_GESTURE_POINTER_NUM_MAX		128
#define FT_GESTURE_POINTER_SIZEOF		4
#define FT_GESTURE_ID_FLAG_SIZE			1
#define FT_GESTURE_POINTER_NUM_FLAG_SIZE	1
/* 6 bytes are taken to mark which gesture is supported in firmware */
#define FT_GESTURE_SET_FLAG_SIZE		6
#define I2C_TRANSFER_MAX_BYTE			255
#define FT_GESTURE_DATA_HEADER	(FT_GESTURE_ID_FLAG_SIZE + \
				FT_GESTURE_POINTER_NUM_FLAG_SIZE + \
				FT_GESTURE_SET_FLAG_SIZE)

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06
#define FT6X36_ID		0x36

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		32768

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0
#define FT_FW_FILE_VENDOR_ID(x)	((x)->data[(x)->size - 1])

#define FT_FW_FILE_MAJ_VER_FT6X36(x)	((x)->data[0x10a])
#define FT_FW_FILE_VENDOR_ID_FT6X36(x)	((x)->data[0x108])

/**
* Application data verification will be run before upgrade flow.
* Firmware image stores some flags with negative and positive value
* in corresponding addresses, we need pick them out do some check to
* make sure the application data is valid.
*/
#define FT_FW_CHECK(x, ts_data) \
	(ts_data->family_id == FT6X36_ID ? \
	(((x)->data[0x104] ^ (x)->data[0x105]) == 0xFF \
	&& ((x)->data[0x106] ^ (x)->data[0x107]) == 0xFF) : \
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& ((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& ((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		48//128	modify by mike.li for tp upgrade fail[2015.05.06][PR994925]
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		30
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"

enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

//add by mike.li for charge mode[2015.04.21][PR983493].
#define USB_CHARGE_DETECT
static bool init_ok = false;
static int ctp_detect_charger_in = 0;

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_psensor_platform_data *psensor_pdata;
	struct ft5x06_gesture_platform_data *gesture_pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
#if defined(FOCALTECH_PWRON_UPGRADE)
	struct delayed_work focaltech_update_work;
	u8 uc_panel_factory_id;
#endif
	u8 fw_vendor_id;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
};

static int ft5x06_ts_start(struct device *dev);
static int ft5x06_ts_stop(struct device *dev);

static struct sensors_classdev __maybe_unused sensors_proximity_cdev = {
	.name = "ft5x06-proximity",
	.vendor = "FocalTech",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static inline bool ft5x06_psensor_support_enabled(void)
{
	return config_enabled(CONFIG_TOUCHSCREEN_FT5X06_PSENSOR);
}

static inline bool ft5x06_gesture_support_enabled(void)
{
	return config_enabled(CONFIG_TOUCHSCREEN_FT5X06_GESTURE);
}

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

#ifdef CONFIG_TOUCHSCREEN_FT5X06_PSENSOR
static void ft5x06_psensor_enable(struct ft5x06_ts_data *data, int enable)
{
	u8 state;
	int ret = -1;

	if (data->client == NULL)
		return;

	ft5x0x_read_reg(data->client, FT_REG_PSENSOR_ENABLE, &state);
	if (enable)
		state |= FT_PSENSOR_ENABLE_MASK;
	else
		state &= ~FT_PSENSOR_ENABLE_MASK;

	ret = ft5x0x_write_reg(data->client, FT_REG_PSENSOR_ENABLE, state);
	if (ret < 0)
		dev_err(&data->client->dev,
			"write psensor switch command failed\n");
	return;
}

static int ft5x06_psensor_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ft5x06_psensor_platform_data *psensor_pdata =
		container_of(sensors_cdev,
			struct ft5x06_psensor_platform_data, ps_cdev);
	struct ft5x06_ts_data *data = psensor_pdata->data;
	struct input_dev *input_dev = data->psensor_pdata->input_psensor_dev;

	mutex_lock(&input_dev->mutex);
	ft5x06_psensor_enable(data, enable);
	psensor_pdata->tp_psensor_data = FT_PSENSOR_ORIGINAL_STATE_FAR;
	if (enable)
		psensor_pdata->tp_psensor_opened = 1;
	else
		psensor_pdata->tp_psensor_opened = 0;
	mutex_unlock(&input_dev->mutex);
	return enable;
}

static int ft5x06_read_tp_psensor_data(struct ft5x06_ts_data *data)
{
	u8 psensor_status;
	char tmp;
	int ret = 1;

	ft5x0x_read_reg(data->client,
			FT_REG_PSENSOR_STATUS, &psensor_status);

	tmp = data->psensor_pdata->tp_psensor_data;
	if (psensor_status == FT_PSENSOR_STATUS_NEAR)
		data->psensor_pdata->tp_psensor_data =
						FT_PSENSOR_FAR_TO_NEAR;
	else if (psensor_status == FT_PSENSOR_STATUS_FAR)
		data->psensor_pdata->tp_psensor_data =
						FT_PSENSOR_NEAR_TO_FAR;

	if (tmp != data->psensor_pdata->tp_psensor_data) {
		dev_dbg(&data->client->dev,
				"%s sensor data changed\n", __func__);
		ret = 0;
	}
	return ret;
}
#else
static int ft5x06_psensor_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	return enable;
}

static int ft5x06_read_tp_psensor_data(struct ft5x06_ts_data *data)
{
	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_GESTURE
static ssize_t ft5x06_gesture_enable_to_set_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			data->gesture_pdata->gesture_enable_to_set);
}

static ssize_t ft5x06_gesture_enable_to_set_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	if (data->suspended)
		return -EINVAL;

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		dev_err(dev, "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}

	if (1 == value)
		data->gesture_pdata->gesture_enable_to_set = 1;
	else
		data->gesture_pdata->gesture_enable_to_set = 0;
	return size;
}

static DEVICE_ATTR(enable, 0664,
		ft5x06_gesture_enable_to_set_show,
		ft5x06_gesture_enable_to_set_store);

static int ft5x06_entry_pocket(struct device *dev)
{
	return ft5x06_ts_stop(dev);
}

static int ft5x06_leave_pocket(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;
	ft5x06_ts_start(dev);

	if (ft5x06_gesture_support_enabled() && device_may_wakeup(dev) &&
		data->pdata->gesture_support &&
		data->gesture_pdata->gesture_enable_to_set) {
		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 1);
		err = enable_irq_wake(data->client->irq);
		if (err)
			dev_err(&data->client->dev,
				"%s: set_irq_wake failed\n", __func__);
	}
	return err;
}

static ssize_t gesture_in_pocket_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			data->gesture_pdata->in_pocket);
}

static ssize_t gesture_in_pocket_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		dev_err(dev, "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	if (1 == value) {
		data->gesture_pdata->in_pocket = 1;
		ft5x06_entry_pocket(dev);
	} else if (0 == value) {
		ft5x06_leave_pocket(dev);
		data->gesture_pdata->in_pocket = 0;
	}
	return size;
}

static DEVICE_ATTR(pocket, 0664,
		gesture_in_pocket_mode_show,
		gesture_in_pocket_mode_store);

static int ft5x06_report_gesture_doubleclick(struct input_dev *ip_dev)
{
	int i;
	for (i = 0; i < 2; i++) {
		input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
		input_report_abs(ip_dev, ABS_MT_POSITION_X,
					FT_GESTURE_DOUBLECLICK_COORD_X);
		input_report_abs(ip_dev, ABS_MT_POSITION_Y,
					FT_GESTURE_DOUBLECLICK_COORD_Y);
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
		input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}
	return 0;
}

static int ft5x06_report_gesture(struct i2c_client *i2c_client,
		struct input_dev *ip_dev)
{
	int i, temp, gesture_data_size;
	int gesture_coord_x, gesture_coord_y;
	int ret = -1;
	short pointnum = 0;
	unsigned char buf[FT_GESTURE_POINTER_NUM_MAX *
			FT_GESTURE_POINTER_SIZEOF + FT_GESTURE_DATA_HEADER];

	buf[0] = FT_REG_GESTURE_OUTPUT;
	ret = ft5x06_i2c_read(i2c_client, buf, 1,
				buf, FT_GESTURE_DATA_HEADER);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	/* FW support doubleclick */
	if (FT_GESTURE_DOUBLECLICK_ID == buf[0]) {
		ft5x06_report_gesture_doubleclick(ip_dev);
		return 0;
	}

	pointnum = (short)(buf[1]) & 0xff;
	gesture_data_size = pointnum * FT_GESTURE_POINTER_SIZEOF +
			FT_GESTURE_DATA_HEADER;
	buf[0] = FT_REG_GESTURE_OUTPUT;
	temp = gesture_data_size / I2C_TRANSFER_MAX_BYTE;
	for (i = 0; i < temp; i++)
		ret = ft5x06_i2c_read(i2c_client, buf, ((i == 0) ? 1 : 0),
			buf + I2C_TRANSFER_MAX_BYTE * i, I2C_TRANSFER_MAX_BYTE);
	ret = ft5x06_i2c_read(i2c_client, buf, ((temp == 0) ? 1 : 0),
			buf + I2C_TRANSFER_MAX_BYTE * temp,
			gesture_data_size - I2C_TRANSFER_MAX_BYTE * temp);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	for (i = 0; i < pointnum; i++) {
		gesture_coord_x = (((s16) buf[FT_GESTURE_DATA_HEADER +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0x0F) << 8 |
				(((s16) buf[FT_GESTURE_DATA_HEADER + 1 +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0xFF);
		gesture_coord_y = (((s16) buf[FT_GESTURE_DATA_HEADER + 2 +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0x0F) << 8 |
				(((s16) buf[FT_GESTURE_DATA_HEADER + 3 +
				(FT_GESTURE_POINTER_SIZEOF * i)]) & 0xFF);
		input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
		input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
		input_report_abs(ip_dev, ABS_MT_POSITION_X, gesture_coord_x);
		input_report_abs(ip_dev, ABS_MT_POSITION_Y, gesture_coord_y);
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}
	input_mt_slot(ip_dev, FT_GESTURE_DEFAULT_TRACKING_ID);
	input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
	input_mt_report_pointer_emulation(ip_dev, false);
	input_sync(ip_dev);

	return 0;
}
#else
static DEVICE_ATTR(pocket, 0664, NULL, NULL);
static DEVICE_ATTR(enable, 0664, NULL, NULL);

static int ft5x06_report_gesture(struct i2c_client *i2c_client,
		struct input_dev *ip_dev)
{
	return 0;
}
#endif


#if defined(USB_CHARGE_DETECT)
void ft5x06_enable_change_scanning_frq(void)
{
	u8 ic_changer_flag = 0;

	if(!ft_g_client)
		return;
	if(init_ok == false)
		return;

	ctp_detect_charger_in = 1;
	ft5x0x_read_reg(ft_g_client, 0x8b, &ic_changer_flag);
	printk("[MIKE]ic_changer_in_flag :%d\n ", ic_changer_flag);
	if (!ic_changer_flag){
		printk("[MIKE]%s: Write %d to 0x8b\n", __func__, ctp_detect_charger_in);
		ft5x0x_write_reg(ft_g_client, 0x8b, ctp_detect_charger_in);
	}
}
EXPORT_SYMBOL(ft5x06_enable_change_scanning_frq);

void ft5x06_disable_change_scanning_frq(void)
{
	u8 ic_changer_flag = 0;

	if(!ft_g_client)
		return;
	if(init_ok == false)
		return;

	ctp_detect_charger_in = 0;
	ft5x0x_read_reg(ft_g_client, 0x8b, &ic_changer_flag);
	printk("[MIKE]ic_changer_in_flag :%d\n ", ic_changer_flag);
	if (ic_changer_flag){
		printk("[MIKE]%s: Write %d to 0x8b\n", __func__, ctp_detect_charger_in);
		ft5x0x_write_reg(ft_g_client, 0x8b, ctp_detect_charger_in);
	}
}
EXPORT_SYMBOL(ft5x06_disable_change_scanning_frq);
#else
 void ft5x06_enable_change_scanning_frq(void)
{
}
EXPORT_SYMBOL(ft5x06_enable_change_scanning_frq);
void ft5x06_disable_change_scanning_frq(void)
{
}
EXPORT_SYMBOL(ft5x06_disable_change_scanning_frq);
#endif

int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	ft5x0x_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft5x0x_write_reg(client, 2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		ft5x0x_read_reg(client, 0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

	msleep(200);
	/*calibration OK */
	msleep(300);
	ft5x0x_write_reg(client, 0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft5x0x_write_reg(client, 2, 0x5);	/*store CLB result */
	msleep(300);
	ft5x0x_write_reg(client, 0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}

static void ft5x06_update_fw_vendor_id(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VENDOR_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0)
		dev_err(&client->dev, "fw vendor id read failed");
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	dev_info(&client->dev, "Firmware version = %x.%x.%x\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, status, num_touches;
	u8 reg, *buf, gesture_is_active;
	bool update_input = false;

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}

	ip_dev = data->input_dev;
	buf = data->tch_data;

	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support &&
		data->psensor_pdata->tp_psensor_opened) {
		rc = ft5x06_read_tp_psensor_data(data);
		if (!rc) {
			if (data->suspended)
				pm_wakeup_event(&data->client->dev,
					FT_PSENSOR_WAKEUP_TIMEOUT);
			input_report_abs(data->psensor_pdata->input_psensor_dev,
					ABS_DISTANCE,
					data->psensor_pdata->tp_psensor_data);
			input_sync(data->psensor_pdata->input_psensor_dev);
			if (data->suspended)
				return IRQ_HANDLED;
		}
		if (data->suspended)
			return IRQ_HANDLED;
	}

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		ft5x0x_read_reg(data->client, FT_REG_GESTURE_ENABLE,
					&gesture_is_active);
		if (gesture_is_active) {
			pm_wakeup_event(&(data->client->dev),
					FT_GESTURE_WAKEUP_TIMEOUT);
			ft5x06_report_gesture(data->client, ip_dev);
			return IRQ_HANDLED;
		}
	}

	/*
	 * Read touch data start from register FT_REG_DEV_MODE.
	 * The touch x/y value start from FT_TOUCH_X_H/L_POS and
	 * FT_TOUCH_Y_H/L_POS in buf.
	 */
	reg = FT_REG_DEV_MODE;
	rc = ft5x06_i2c_read(data->client, &reg, 1, buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;

		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		} else {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	return IRQ_HANDLED;
}

static int ft5x06_gpio_configure(struct ft5x06_ts_data *data, bool on)
{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->irq_gpio)) {
			err = gpio_request(data->pdata->irq_gpio,
						"ft5x06_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->irq_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
			err = gpio_request(data->pdata->reset_gpio,
						"ft5x06_reset_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"reset gpio request failed");
				goto err_irq_gpio_dir;
			}

			err = gpio_direction_output(data->pdata->reset_gpio, 0);
			if (err) {
				dev_err(&data->client->dev,
				"set_direction for reset gpio failed\n");
				goto err_reset_gpio_dir;
			}
			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		}

		return 0;
	} else {
		if (gpio_is_valid(data->pdata->irq_gpio))
			gpio_free(data->pdata->irq_gpio);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */
			//modify by mike.li for save leakage current.[2015.03.13][PR949547]
			//err = gpio_direction_input(data->pdata->reset_gpio);
			//if (err) {
			//	dev_err(&data->client->dev,
			//		"unable to set direction for gpio "
			//		"[%d]\n", data->pdata->irq_gpio);
			//}
			gpio_free(data->pdata->reset_gpio);
		}

		return 0;
	}

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	return err;
}

static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	ft5x06_data->pinctrl_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_active)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_active);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_suspend);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_release
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_release)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_release);
		dev_dbg(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ft5x06_data->ts_pinctrl);
err_pinctrl_get:
	ft5x06_data->ts_pinctrl = NULL;
	return retval;
}

#ifdef CONFIG_PM
static int ft5x06_ts_start(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

#if 0	//modify by mike.li for save leakage current.[2015.03.13][PR949547]
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}

	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
				data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}
#endif
	err = ft5x06_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in resue state\n");
		goto err_gpio_configuration;
	}

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(data->pdata->soft_rst_dly);

	enable_irq(data->client->irq);
	data->suspended = false;

	return 0;

err_gpio_configuration:
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}
	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err)
			dev_err(dev, "power off failed");
	} else {
		err = ft5x06_power_on(data, false);
		if (err)
			dev_err(dev, "power off failed");
	}
	return err;
}

static int ft5x06_ts_stop(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2];
	int i, err;

	disable_irq(data->client->irq);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}
#if 0	//modify by mike.li for save leakage current.[2015.03.13][PR949547]

	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}

	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_suspend);
		if (err < 0)
			dev_err(dev, "Cannot get suspend pinctrl state\n");
	}
#endif

	err = ft5x06_gpio_configure(data, false);
	if (err < 0) {
		dev_err(&data->client->dev,
			"failed to put gpios in suspend state\n");
		goto gpio_configure_fail;
	}

	data->suspended = true;

	return 0;

gpio_configure_fail:
	if (data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0)
			dev_err(dev, "Cannot get active pinctrl state\n");
	}
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err)
			dev_err(dev, "power on failed");
	} else {
		err = ft5x06_power_on(data, true);
		if (err)
			dev_err(dev, "power on failed");
	}

#if 0	//modify by mike.li for save leakage current.[2015.03.13][PR949547]
pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
#endif

	enable_irq(data->client->irq);
	return err;
}

static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support &&
		device_may_wakeup(dev) &&
		data->psensor_pdata->tp_psensor_opened) {

		err = enable_irq_wake(data->client->irq);
		if (err)
			dev_err(&data->client->dev,
				"%s: set_irq_wake failed\n", __func__);
		data->suspended = true;
		return err;
	}

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support &&
		device_may_wakeup(dev) &&
		data->gesture_pdata->gesture_enable_to_set) {

		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 1);
		err = enable_irq_wake(data->client->irq);
		if (err)
			dev_err(&data->client->dev,
				"%s: set_irq_wake failed\n", __func__);
		data->suspended = true;
		return err;
	}

	return ft5x06_ts_stop(dev);
}

static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support &&
		device_may_wakeup(dev) &&
		data->psensor_pdata->tp_psensor_opened) {
		err = disable_irq_wake(data->client->irq);
		if (err)
			dev_err(&data->client->dev,
				"%s: disable_irq_wake failed\n",
				__func__);
		data->suspended = false;
		return err;
	}

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support &&
		device_may_wakeup(dev) &&
		!(data->gesture_pdata->in_pocket) &&
		data->gesture_pdata->gesture_enable_to_set) {

		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 0);
		err = disable_irq_wake(data->client->irq);
		if (err)
			dev_err(dev, "%s: disable_irq_wake failed\n",
				__func__);
		data->suspended = false;
		return err;
	}

	err = ft5x06_ts_start(dev);
	if (err < 0)
		return err;

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support &&
		device_may_wakeup(dev) &&
		data->gesture_pdata->in_pocket &&
		data->gesture_pdata->gesture_enable_to_set) {

		ft5x0x_write_reg(data->client, FT_REG_GESTURE_ENABLE, 0);
		err = disable_irq_wake(data->client->irq);
		if (err)
			dev_err(dev, "%s: disable_irq_wake failed\n",
				__func__);
		data->suspended = false;
		data->gesture_pdata->in_pocket = 0;
	}

	#if defined(USB_CHARGE_DETECT)
	if (ctp_detect_charger_in) {
		printk("[MIKE]: enable charge mode from resume\n");
		ft5x06_enable_change_scanning_frq();
	}
	#endif

	return 0;
}

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};

#else
static int ft5x06_ts_suspend(struct device *dev)
{
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	return 0;
}

#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft5x06_ts_resume(&ft5x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

//add by mike.li for after ctp fw upgrade fail can not read vender id.[2015.05.06][PR995189]
static int ft5x06_fw_read_project_setting(struct i2c_client *client)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[4] = {0}, r_buf[8] = {0};
	int i, j;
	u8 is_5336_new_bootloader = false;

	dev_info(&client->dev, "ft5x06_fw_read_project_setting start\n");

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID
			|| ts_data->family_id == FT6X36_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	
 	/*--------- read current project setting  ---------- */
        /*set read start address */
	w_buf[0] = 0x3;
	w_buf[1] = 0x0;
	w_buf[2] = 0x07;
	w_buf[3] = 0xB0;

	ft5x06_i2c_read(client, w_buf, 4, r_buf, 8);
	dev_info(&client->dev,"[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
			r_buf[0], r_buf[2], r_buf[4]);

	/********* reset the new FW***********************/
	w_buf[0] = 0x07;
	ft5x06_i2c_write(client, w_buf, 1);

	msleep(200);

	ts_data->uc_panel_factory_id = r_buf[4];
	dev_info(&client->dev, "uc_panel_factory_id is 0x%x\n", ts_data->uc_panel_factory_id);
	return ts_data->uc_panel_factory_id;
}

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;

	dev_info(&client->dev, "Firmware upgrade start\n");

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID
			|| ts_data->family_id == FT6X36_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min, fw_file_vendor_id;
	bool fw_upgrade = false;
	int update_retry = 0;

retry_update:
	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zu)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	if (data->family_id == FT6X36_ID) {
		fw_file_maj = FT_FW_FILE_MAJ_VER_FT6X36(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID_FT6X36(fw);
	} else {
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
	}
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: 0x%x.%x.%x", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: 0x%x.%x.%x", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force)
	{
		fw_upgrade = true;
		dev_info(dev, "CTP will force upgrade firmware!!!!!!\n");
	}
	else if ((data->fw_ver[0] < fw_file_maj) &&
		(data->fw_vendor_id == fw_file_vendor_id || (0x11 == data->fw_vendor_id)))
	{
		fw_upgrade = true;
	//add by mike.li for after ctp fw upgrade fail can not read vender id.[2015.05.06][PR995189]
	} else if (0x0 == data->fw_vendor_id || 0xFF == data->fw_vendor_id) {
		ft5x06_fw_read_project_setting(data->client);
		if (data->uc_panel_factory_id == fw_file_vendor_id)
			fw_upgrade = true;
	}

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (FT_FW_CHECK(fw, data)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0) {
			dev_err(dev, "update failed (%d). try (%d) later...\n", rc, update_retry);
			if (0 == update_retry) {
				update_retry++;
				goto retry_update;
			}
		}			
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(fw);
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
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

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		//return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	pdata->psensor_support = of_property_read_bool(np,
						"focaltech,psensor-support");

	pdata->gesture_support = of_property_read_bool(np,
						"focaltech,gesture-support");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#if defined(FOCALTECH_PWRON_UPGRADE)
static void ft_init_update_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct ft5x06_ts_data *ts;
	struct device *dev;

	delay_work = to_delayed_work(work);
	ts = container_of(delay_work, struct ft5x06_ts_data, focaltech_update_work);
	dev = &ts->input_dev->dev;

	mutex_lock(&ts->input_dev->mutex);
	#ifdef MINI_MODE_TO_CTP
	ft5x06_fw_upgrade(dev, true);
	#else
	ft5x06_fw_upgrade(dev, false);
	#endif
	mutex_unlock(&ts->input_dev->mutex);
}

u8 ft_init_update_proc(struct ft5x06_ts_data *ts)
{
	dev_info(&ts->client->dev, "Ready to run update work.");

	INIT_DELAYED_WORK(&ts->focaltech_update_work, ft_init_update_work);
	schedule_delayed_work(&ts->focaltech_update_work,
		msecs_to_jiffies(3000));

	return 0;
}
#endif

static ssize_t firm_ver_show (struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{

   u8 reg_ver;
   u8 reg_vendor;
   u8 ver_value,vendor_value;
   int err;

   reg_ver = FT_REG_FW_VER;
   err = ft5x06_i2c_read(ft_g_client, &reg_ver, 1, &ver_value, 1);
   if (err < 0) {
       pr_err( "TP FW version read failure\n");
       return sprintf ( buf, "can't read firmware version \n" );
   }
   pr_err("0xA6=0x%x\n",ver_value);
   reg_vendor = 0xA8;
   err = ft5x06_i2c_read(ft_g_client, &reg_vendor, 1, &vendor_value, 1);
   if (err < 0) {
      pr_err( "TP FW version read failure\n");
      return sprintf ( buf, "ft irmware version(0xA6) is 0x%x\n can't read tp moudule  version \n" ,ver_value);
   }
      pr_err("0xA8=0x%x\n",vendor_value);

   return sprintf ( buf, "ft TP module  (0xA8)is 0x%x ,fimware version(0xA6) is 0x%x\n",vendor_value,ver_value);
}

#if defined(FT_ITO_TEST)
#define FT_INI_FILEPATH	"/system/etc/ft_rawdata.ini"
#define FT_RAWDATA_SIZE		2*1024

static int ft5x06_GetInISize(void)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;

	if (NULL == pfile)
		pfile = filp_open(FT_INI_FILEPATH, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		printk("ft5x06_GetInISize : error occured while opening file %s.\n", FT_INI_FILEPATH);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x06_ReadInIData(char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	loff_t pos;
	mm_segment_t old_fs;

	if (NULL == pfile)
		pfile = filp_open(FT_INI_FILEPATH, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		printk("ft5x06_GetInISize error occured while opening file %s.\n", FT_INI_FILEPATH);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

static int ft5x06_get_testparam_from_ini(void)
{
	char *filedata = NULL;
	int inisize = ft5x06_GetInISize();

	printk("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		printk("%s ERROR:Get firmware size failed\n",__func__);
		return -EIO;
	}

	filedata = kmalloc(inisize + 1, GFP_ATOMIC);

	
	if (ft5x06_ReadInIData(filedata)) {
		printk("%s() - ERROR: request_firmware failed\n",__func__);
		kfree(filedata);
		return -EIO;
	} else {
		printk("%s successful\n",__func__);
	}

	SetParamData(filedata);
	return 0;
}

int FTS_I2c_Read(unsigned char * wBuf, int wLen, unsigned char *rBuf, int rLen)
{
	if(NULL == ft_g_client)
	{
		return -1;
	}

	return ft5x06_i2c_read(ft_g_client, wBuf, wLen, rBuf, rLen);

}

int FTS_I2c_Write(unsigned char * wBuf, int wLen)
{	
	if(NULL == ft_g_client)
	{
		return -1;
	}	

	return ft5x06_i2c_write(ft_g_client, wBuf, wLen);
}

static ssize_t ft5x06_rawdata_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
    ssize_t ret_count = 0;
    char *databuf = NULL;

    databuf = kmalloc(FT_RAWDATA_SIZE, GFP_ATOMIC);

    if(databuf == NULL)
    {
        ret_count = sprintf(buf, "Alloc memory failed!\n");
        return ret_count;
    }

    memset(databuf, 0 , FT_RAWDATA_SIZE);

    mutex_lock(&g_rawdata_mutex);

    Init_I2C_Write_Func(FTS_I2c_Write);
    Init_I2C_Read_Func(FTS_I2c_Read);

    if(ft5x06_get_testparam_from_ini() <0)
    {
        printk("%s get testparam from ini failure\n",__func__);
    }
    else 
    {
        StartTestTP();
        FreeTestParamData();

        ret_count = focal_save_scap_sample(databuf, FT_RAWDATA_SIZE);
        memcpy(buf, databuf, ret_count);
    }

    mutex_unlock(&g_rawdata_mutex);

    kfree(databuf);
    databuf = NULL;

    return ret_count;
}

static ssize_t ft5x06_rawdata_result_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
    char *s = buf;

    mutex_lock(&g_rawdata_mutex);

    Init_I2C_Write_Func(FTS_I2c_Write);
    Init_I2C_Read_Func(FTS_I2c_Read);

    if(ft5x06_get_testparam_from_ini() <0)
    {
        printk("%s get testparam from ini failure\n",__func__);
    }
    else 
    {
        if(true == StartTestTP())
        {
            printk("%s ****PASS***\n",__func__);
            s += sprintf(s, "PASS\n");
        }
        else
        {
            printk("%s ***NG***\n",__func__);
            s += sprintf(s, "NG\n");
        }

        FreeTestParamData();
    }

    mutex_unlock(&g_rawdata_mutex);

    return (s-buf);
}

static struct kobj_attribute ft5x06_rawdata_attr = {
        .attr = {
                .name = "rawdata",
                .mode = S_IRUGO,
        },
        .show = &ft5x06_rawdata_show,
};

static struct kobj_attribute ft5x06_ftsscaptest_attr = {
	.attr = {
		.name = "rd_result",
		.mode = S_IRUGO,
	},
	.show = &ft5x06_rawdata_result_show,
};

static struct kobj_attribute ft5x06_firm_ver_attr = {
        .attr = {
                .name = "firm_ver",
                .mode = S_IRUGO,
        },
        .show = &firm_ver_show,
};

static struct attribute *ft5x06_rawdata_properties_attrs[] = {
	&ft5x06_rawdata_attr.attr,
	&ft5x06_ftsscaptest_attr.attr,
	&ft5x06_firm_ver_attr.attr,
	NULL,
};

static struct attribute_group ft5x06_rawdata_properties_attr_group = {
        .attrs = ft5x06_rawdata_properties_attrs,
};

int ft5x06_rawdata_attr_create(void)
{ 
	static struct kobject *ft5x06_rawdata_properties_kobj;
	int rc = 0;

	ft5x06_rawdata_properties_kobj = kobject_create_and_add("CTP", NULL);

	if (ft5x06_rawdata_properties_kobj)
	{
		rc = sysfs_create_group(ft5x06_rawdata_properties_kobj,&ft5x06_rawdata_properties_attr_group);
	}

	if (!ft5x06_rawdata_properties_kobj || rc)
	{
		pr_err("%s: failed to create rawdata\n", __func__);
	}

	return 0;
}
#endif


#if defined(FT_PROC_DEBUG)
#define FTS_PACKET_LENGTH        128
#define PROC_UPGRADE              	0
#define PROC_READ_REGISTER          	1
#define PROC_WRITE_REGISTER        	2
#define PROC_AUTOCLB                	4
#define PROC_UPGRADE_INFO           	5
#define PROC_WRITE_DATA               	6
#define PROC_READ_DATA                 	7

#define PROC_NAME    "ft5x0x-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *ft5x0x_proc_entry;


/*interface of write proc*/
static ssize_t ft5x0x_debug_write(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
      struct i2c_client *client = ft_g_client;
      unsigned char writebuf[FTS_PACKET_LENGTH];
      int buflen = count;
      int writelen = 0;
      int ret = 0;

      if (copy_from_user(writebuf, (void __user *)buffer, buflen)) {
           dev_err(&client->dev, "%s:copy from user error\n", __func__);
           return -EFAULT;
      }

	proc_operate_mode = writebuf[0];
	printk("proc_operate_mode = %d\n",proc_operate_mode);
      switch (proc_operate_mode) {
      case PROC_READ_REGISTER:
	   printk("%s,%d:PROC_READ_REGISTER\n",__func__,__LINE__);
           writelen = 1;
           ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
           if (ret < 0) {
                 dev_err(&client->dev, "%s:write iic error\n", __func__);
                 return ret;
           }
           break;
      case PROC_WRITE_REGISTER:
	   printk("%s,%d:PROC_WRITE_REGISTER\n",__func__,__LINE__);
           writelen = 2;
           ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
           if (ret < 0) {
                 dev_err(&client->dev, "%s:write iic error\n", __func__);
                 return ret;
           }
           break;
      case PROC_AUTOCLB:
	   printk("%s,%d:PROC_AUTOCLB\n",__func__,__LINE__);
           printk("%s: autoclb\n", __func__);
           fts_ctpm_auto_clb(client);
           break;
      case PROC_READ_DATA:
      case PROC_WRITE_DATA:
	   printk("%s,%d:PROC_READ_DATA,PROC_WRITE_DATA\n",__func__,__LINE__);
           writelen = count - 1;
           ret = ft5x06_i2c_write(client, writebuf + 1, writelen);
           if (ret < 0) {
                 dev_err(&client->dev, "%s:write iic error\n", __func__);
                 return ret;
           }
           break;
      default:
	   printk("%s,%d:default\n",__func__,__LINE__);
           break;
      }

      return count;
}

/*interface of read proc*/
static ssize_t ft5x0x_debug_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
      struct i2c_client *client = ft_g_client;
      int ret = 0;
      unsigned char buf[1000];	//PAGE_SIZE
      int num_read_chars = 0;
      int readlen = 0;
      u8 regvalue = 0x00, regaddr = 0x00;

      switch (proc_operate_mode) {
      case PROC_UPGRADE:
           /*after calling ft5x0x_debug_write to upgrade*/
	   printk("%s,%d:PROC_UPGRADE\n",__func__,__LINE__);
           regaddr = 0xA6;
           ret = ft5x0x_read_reg(client, regaddr, &regvalue);
           if (ret < 0)
                 num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
           else
                 num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
           break;
      case PROC_READ_REGISTER:
           readlen = 1;
           ret = ft5x06_i2c_read(client, NULL, 0, buf, readlen);
           if (ret < 0) {
                 dev_err(&client->dev, "%s:read iic error\n", __func__);
                 return ret;
           }
		   printk("%s,%d:PROC_READ_REGISTER, buf = %c\n",__func__,__LINE__,*buf);
           num_read_chars = 1;
           break;
      case PROC_READ_DATA:
	   printk("%s,%d:PROC_READ_DATA\n",__func__,__LINE__);
           readlen = size;
           ret = ft5x06_i2c_read(client, NULL, 0, buf, readlen);
           if (ret < 0) {
                 dev_err(&client->dev, "%s:read iic error\n", __func__);
                 return ret;
           }

           num_read_chars = readlen;
           break;
      case PROC_WRITE_DATA:
	   printk("%s,%d:PROC_WRITE_DATA\n",__func__,__LINE__);
           break;
      default:
	   printk("%s,%d:default\n",__func__,__LINE__);
           break;
      }

      memcpy(page, buf, num_read_chars);

      return num_read_chars;
}

static const struct file_operations ft5x0x_debug_ops = {
    .owner = THIS_MODULE,
    .read = ft5x0x_debug_read,
    .write = ft5x0x_debug_write,
};

static int ft5x0x_create_apk_debug_channel(struct i2c_client * client)
{
      ft5x0x_proc_entry = proc_create(PROC_NAME, 0777, NULL,&ft5x0x_debug_ops);

      if (NULL == ft5x0x_proc_entry) {
           dev_err(&client->dev, "Couldn't create proc entry!\n");
           return -ENOMEM;
      } else {
           dev_info(&client->dev, "Create proc entry success!\n");
//           ft5x0x_proc_entry->data = client;
  //         ft5x0x_proc_entry->write_proc = ft5x0x_debug_write;
 //          ft5x0x_proc_entry->read_proc = ft5x0x_debug_read;
      }
      return 0;
}

static void ft5x0x_release_apk_debug_channel(void)
{
      if (ft5x0x_proc_entry)
           remove_proc_entry(PROC_NAME, NULL);
}
#endif

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_psensor_platform_data *psensor_pdata;
	struct ft5x06_gesture_platform_data *gesture_pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct input_dev *psensor_input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
	if (!data->tch_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ft_g_client = client;

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	err = ft5x06_ts_pinctrl_init(data);
	if (!err && data->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
		}
	}

	err = ft5x06_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure the gpios\n");
		goto err_gpio_req;
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_gpio;
	}

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_gpio;
	}

	data->family_id = pdata->family_id;

	mutex_init(&g_rawdata_mutex);

#if defined(FOCALTECH_PWRON_UPGRADE)
	err = ft_init_update_proc(data);
	if (err < 0) {
		dev_err(&client->dev,
				"GTP Create firmware update thread error.\n");
//			goto exit_power_off;
	}
#endif

	err = request_threaded_irq(client->irq, NULL,
				ft5x06_ts_interrupt,
	/*
	* the interrupt trigger mode will be set in Device Tree with property
	* "interrupts", so here we just need to set the flag IRQF_ONESHOT
	*/
				IRQF_ONESHOT,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_gpio;
	}

	if (data->pdata->psensor_support && data->pdata->gesture_support) {
		dev_err(&client->dev,
			"Unsupport psensor & gesture at the same time\n");
		goto irq_free;
	}
	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support) {
		device_init_wakeup(&client->dev, 1);
		psensor_pdata = devm_kzalloc(&client->dev,
				sizeof(struct ft5x06_psensor_platform_data),
				GFP_KERNEL);
		if (!psensor_pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			goto irq_free;
		}
		data->psensor_pdata = psensor_pdata;

		psensor_input_dev = input_allocate_device();
		if (!psensor_input_dev) {
			dev_err(&data->client->dev,
				"Failed to allocate device\n");
			goto free_psensor_pdata;
		}

		__set_bit(EV_ABS, psensor_input_dev->evbit);
		input_set_abs_params(psensor_input_dev,
					ABS_DISTANCE, 0, 1, 0, 0);
		psensor_input_dev->name = "proximity";
		psensor_input_dev->id.bustype = BUS_I2C;
		psensor_input_dev->dev.parent = &data->client->dev;
		data->psensor_pdata->input_psensor_dev = psensor_input_dev;

		err = input_register_device(psensor_input_dev);
		if (err) {
			dev_err(&data->client->dev,
				"Unable to register device, err=%d\n", err);
			goto free_psensor_input_dev;
		}

		psensor_pdata->ps_cdev = sensors_proximity_cdev;
		psensor_pdata->ps_cdev.sensors_enable =
					ft5x06_psensor_enable_set;
		psensor_pdata->data = data;

		err = sensors_classdev_register(&client->dev,
						&psensor_pdata->ps_cdev);
		if (err)
			goto unregister_psensor_input_device;
	}

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		device_init_wakeup(&client->dev, 1);
		gesture_pdata = devm_kzalloc(&client->dev,
				sizeof(struct ft5x06_gesture_platform_data),
				GFP_KERNEL);
		if (!gesture_pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			goto free_psensor_class_sysfs;
		}
		data->gesture_pdata = gesture_pdata;
		gesture_pdata->data = data;

		gesture_pdata->gesture_class =
					class_create(THIS_MODULE, "gesture");
		if (IS_ERR(gesture_pdata->gesture_class)) {
			err = PTR_ERR(gesture_pdata->gesture_class);
			dev_err(&client->dev, "Failed to create class.\n");
			goto free_gesture_pdata;
		}

		gesture_pdata->dev = device_create(gesture_pdata->gesture_class,
				NULL, 0, NULL, "gesture_ft5x06");
		if (IS_ERR(gesture_pdata->dev)) {
			err = PTR_ERR(gesture_pdata->dev);
			dev_err(&client->dev, "Failed to create device.\n");
			goto free_gesture_class;
		}

		dev_set_drvdata(gesture_pdata->dev, data);
		err = device_create_file(gesture_pdata->dev,
					&dev_attr_enable);
		if (err) {
			dev_err(gesture_pdata->dev,
					"sys file creation failed\n");
			goto free_gesture_dev;
		}
		err = device_create_file(gesture_pdata->dev,
					&dev_attr_pocket);
		if (err) {
			dev_err(gesture_pdata->dev,
					"sys file creation failed\n");
			goto free_enable_sys;
		}
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_pocket_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);
	ft5x06_update_fw_vendor_id(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

#if defined(FT_ITO_TEST)
	ft5x06_rawdata_attr_create();
#endif

#if defined(FT_PROC_DEBUG)
	if (ft5x0x_create_apk_debug_channel(client) < 0)
		ft5x0x_release_apk_debug_channel();
#endif

	init_ok = true;

	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
free_pocket_sys:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		device_remove_file(&client->dev, &dev_attr_pocket);
free_enable_sys:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		device_remove_file(&client->dev, &dev_attr_enable);
free_gesture_dev:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		device_destroy(gesture_pdata->gesture_class, 0);
free_gesture_class:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support)
		class_destroy(gesture_pdata->gesture_class);
free_gesture_pdata:
	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		devm_kfree(&client->dev, gesture_pdata);
		data->gesture_pdata = NULL;
	}
free_psensor_class_sysfs:
	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support)
		sensors_classdev_unregister(&psensor_pdata->ps_cdev);
unregister_psensor_input_device:
	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support)
		input_unregister_device(data->psensor_pdata->input_psensor_dev);
free_psensor_input_dev:
	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support)
		input_free_device(data->psensor_pdata->input_psensor_dev);
free_psensor_pdata:
	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support) {
		devm_kfree(&client->dev, psensor_pdata);
		data->psensor_pdata = NULL;
	}
irq_free:
	if ((ft5x06_psensor_support_enabled() &&
		data->pdata->psensor_support) ||
		(ft5x06_gesture_support_enabled() &&
		data->pdata->gesture_support))

		device_init_wakeup(&client->dev, 0);
	free_irq(client->irq, data);
free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_gpio_req:
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
unreg_inputdev:
	input_unregister_device(input_dev);
free_inputdev:
	input_free_device(input_dev);
	input_dev = NULL;
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	int retval;

	if (ft5x06_gesture_support_enabled() && data->pdata->gesture_support) {
		device_init_wakeup(&client->dev, 0);
		device_remove_file(&client->dev, &dev_attr_pocket);
		device_remove_file(&client->dev, &dev_attr_enable);
		device_destroy(data->gesture_pdata->gesture_class, 0);
		class_destroy(data->gesture_pdata->gesture_class);
		devm_kfree(&client->dev, data->gesture_pdata);
		data->gesture_pdata = NULL;
	}

	if (ft5x06_psensor_support_enabled() && data->pdata->psensor_support) {

		device_init_wakeup(&client->dev, 0);
		sensors_classdev_unregister(&data->psensor_pdata->ps_cdev);
		input_unregister_device(data->psensor_pdata->input_psensor_dev);
		input_free_device(data->psensor_pdata->input_psensor_dev);
		devm_kfree(&client->dev, data->psensor_pdata);
		data->psensor_pdata = NULL;
	}

	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (retval < 0)
				pr_err("failed to select release pinctrl state\n");
		}
	}

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

#if defined(FT_PROC_DEBUG)
	ft5x0x_release_apk_debug_channel();
#endif

	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
