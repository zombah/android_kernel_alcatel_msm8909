/**
void adux1050_usb_calib(void);
*\mainpage
* ADUX1050 Generic Controller Driver
\n
* @copyright 2014 Analog Devices Inc.
\n
* Licensed under the GPL version 2 or later.
* \date      August-2014
* \version   Driver 1.0
* \version   Linux 3.4.0 and above
* \version   Android 4.4.2 [KK]
*/

/**
* \file adux1050.c
* This file is the core driver part of ADUX1050 Capacitive sensor.
* It also has routines for interrupt handling,
* suspend, resume, initialization routines etc.
<br>
* ADUX1050 Generic Controller Driver
<br>
* Copyright 2014 Analog Devices Inc.
<br>
* Licensed under the GPL version 2 or later.
*/

//#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/wait.h>
#include "adux1050.h"
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/switch.h>
#include "psensor.h"
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <../fs/proc/internal.h>
#include <linux/uaccess.h>

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) "%s: " fmt, __func__
#else 
#define pr_fmt(fmt) "%s: " fmt, __func__
#endif

#ifdef pr_info
#undef pr_info
#define pr_info(fmt, ...) \
	printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_info(fmt, ...) \
	printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifdef dev_info
#undef dev_info
#define dev_info(dev, fmt, ...) \
        printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#else
#define dev_info(dev, fmt, ...) \
        printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) \
	printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...) \
	printk(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#endif

/**
 This is the path for the DAC calibration data file storage
 The File name prefix and the location of the file can be changed
 as per the platform used.
*/
/*
#ifdef CONFIG_USE_FILP
#undef CONFIG_USE_FILP
#endif

#ifndef CONFIG_EVAL
#define CONFIG_EVAL
#endif
*/

#ifdef CONFIG_USE_FILP
#undef CONFIG_USE_FILP
#endif

#ifndef CONFIG_EVAL
#define CONFIG_EVAL
#endif

#define CAL_DATA_FILE_PATH	"/data/misc/adux1050_cal_data"

#define CONFIG_ADUX1050_DEBUG

#ifdef CONFIG_ADUX1050_DEBUG
#define ADUX1050_DRIVER_DBG(format, arg...) pr_info("[ADUX1050]:"format,\
						    ## arg)
#else
#define ADUX1050_DRIVER_DBG(format, arg...) do { if (0); } while (0)
#endif


#ifdef USE_TRANCEBLITY
#define TRANCEBILITY_BASE 0x8606400
#define TRANCEBILITY_LENGTH 14
//static void write_trace(struct adux1050_chip *adux1050);
#endif
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.21*/
static struct proc_dir_entry *proc_dir;
static struct proc_dir_entry *adux1050_enable_proc;
u8 *p_sendevent = NULL;
/* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.21*/

//#define ADUX_VDD_MIN_UV       2850000
//#define ADUX_VDD_MAX_UV       2850000
#define ADUX_VIO_MIN_UV       1800000
#define ADUX_VIO_MAX_UV       1800000

struct i2c_driver adux1050_i2c_driver;
static const struct dev_pm_ops adux1050_dev_pm_ops;

#define COMPATIBLE_NEEDED
struct psensor_data *psensor_data;
extern int iqs128_init(struct device *dev);
static int adux_power_off(struct adux1050_chip *adux1050);
static int adux_power_on(struct adux1050_chip *adux1050);
extern int iqs_pinctrl_init(struct device *dev);
extern struct iqs_pinctrl_info iqs128_pctrl;
//#define ENVIRONMENT_CALIBRATION

#define STATE_NEAR 0
#define STATE_FAR  1
//static inline void indicate_state(struct adux1050_chip *adux1050, int val);
/**
 Local platform data used if no platform data is found in board file or
 Device tree.
i*/
static struct adux1050_platform_data local_plat_data = {
            .init_regs = {
                        0x00010242,   0x00020538,   0x000301EE,   0x00050F77,
                        0x00061908,  0x000700F0,   0x00080096,   0x00090000,
                        0x000a0003,   0x00798000,
            },
            .req_stg0_base = 10000,
            .req_stg1_base = 20000,
            .req_stg2_base = 30000,
            .req_stg3_base = 40000,
};
#ifdef ENVIRONMENT_CALIBRATION
u16 calib_data_cache[20];
#endif
/**
* \fn int adux1050_i2c_write(struct device *dev, u8 reg,u16 *data, u16 data_cnt)
* Writes to the device register through I2C interface.
* Used to write the data to the I2C client's Register through the i2c protocol
* Used i2c_tranfer api's for the bus transfer
@param dev The i2c client's device structure
@param reg The register address to be written
@param data The data buffer which holds the data to be written to the device
@param data_cnt The number of data to be written to the device from buffer.
@return Number of messages transfered, default 2

@see adux1050_i2c_read
*/
static int adux1050_i2c_write(struct device *dev, u8 reg,
		u16 *data, u16 data_cnt)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 device_addr = client->addr;
	u16 loop_cnt = 0;
	u8 tx[(MAX_ADUX1050_WR_LEN*sizeof(short)) + 1] = {0};
	u16 *head;
	s32 ret = -EIO;
	struct i2c_msg adux1050_wr_msg = {
		.addr = device_addr,
		.buf = (u8 *)tx,
		.len = ((data_cnt*sizeof(data_cnt))+1),
		.flags = 0,
	};
	if (!data)
		return -EINVAL;
	tx[0] = reg;
	head = (unsigned short *)&tx[1];
	for (loop_cnt = 0; loop_cnt < data_cnt; loop_cnt++)
		*(head++) = cpu_to_be16(*(data++));

	for (loop_cnt = 0; loop_cnt < I2C_RETRY_CNT; loop_cnt++) {
		ret = i2c_transfer(client->adapter, &adux1050_wr_msg, 1);
		if (unlikely(ret < 1))
			dev_err(&client->dev, "I2C write error %d\n", ret);
		else
			break;
	}
	return ret;
}

/**
* \fn int adux1050_i2c_read(struct device *dev, u8 reg,u16 *data, u16 data_cnt)
* This is used to read the data from the ADUX1050's register through
I2C interface
* This function uses i2c protocol and its api's to read data from register
@param dev The i2c client device Structure.
@param reg The register address to be read.
@param data The buffer's pointer to store the register's value.
@param data_cnt The number of registers to be read.
@return The number of messages transfered as an integer

@see adux1050_i2c_write
*/
static int adux1050_i2c_read(struct device *dev, u8 reg,
			     u16 *data, u16 data_cnt)
{
	struct i2c_client *client = to_i2c_client(dev);
	u16 loop_cnt = 0;
	u16 rx[MAX_ADUX1050_WR_LEN] = {};
	s8  device_addr = client->addr;
	s32 ret = 0;
	struct i2c_msg adux1050_rd_msg[I2C_WRMSG_LEN] = {
			{
				.addr = device_addr,
				.buf = (u8 *)&reg,
				.len = sizeof(reg),
				.flags = 0,
			},
			{
				.addr = device_addr,
				.buf = (u8 *)rx,
				.len = data_cnt * sizeof(short),
				.flags = I2C_M_RD,
			}
	};
	for (loop_cnt = 0; loop_cnt < I2C_RETRY_CNT; loop_cnt++) {
		ret = i2c_transfer(client->adapter, adux1050_rd_msg,
				   I2C_WRMSG_LEN);
		if (unlikely(ret < I2C_WRMSG_LEN)) {
			dev_err(dev, "[ADUX1050]: I2C READ error %d\n", ret);
			if (loop_cnt >= (I2C_RETRY_CNT - 1))
				memset(data, 0, data_cnt * sizeof(short));
		} else {
			for (loop_cnt = 0; loop_cnt < data_cnt; loop_cnt++)
				data[loop_cnt] = be16_to_cpu(rx[loop_cnt]);
			break;
		}
	}
	return ret;
}

/**
* \fn inline u16 set_dac_offset(s16 new_offset_value)
* Function to set the DAC positive and negative offset based on given offset
@param new_offset_value value to be set as the offset.
@return Combined +ve and -ve value to be set to the DAC_OFFSET_STGx register
*/
static inline u16 set_dac_offset(s16 new_offset_value)
{
	u16 offset_val = 0;
	if (new_offset_value >= 0)
		offset_val = ST_POS_DAC_OFFSET(new_offset_value);
	else
		offset_val = ST_NEG_DAC_OFFSET(new_offset_value);

	return offset_val;
}

/**
* \fn static inline u16 set_swap_state(struct adux1050_chip *adux1050, u16 stg_num, u16 *swap_state, s16 curr_val)
* Function to set the swap bits based on the calibration DAC offset
@param adux1050 chip structure of ADUX1050 driver.
@param stg_num The stage to which swap is to be done for DAC offset value.
@param swap_state The swap state as set in control register
	[ctrl reg holds the swap state]
@param curr_val Current value of DAC_offest for the stage provided [stg_num]
@return Zero on success.
*/
static inline u16 set_swap_state(struct adux1050_chip *adux1050, u16 stg_num,
		   u16 *swap_state, s16 curr_val) {
	u16 err = 0;
	if (!swap_state)
		return -EINVAL;
	if (curr_val > (MAX_OFFSET/2)) {
		*swap_state = ((*swap_state & CLR_POS_SWAP) | SET_NEG_SWAP);
		err = adux1050->write(adux1050->dev, GET_CONFIG_REG(stg_num),
				swap_state, DEF_WR);
		dev_dbg(adux1050->dev, "%s - Swap set %x",
			__func__, *swap_state);
	} else if (curr_val < -(MAX_OFFSET/2)) {
		*swap_state = ((*swap_state & CLR_NEG_SWAP) | SET_POS_SWAP);
		err = adux1050->write(adux1050->dev, GET_CONFIG_REG(stg_num),
				      swap_state, DEF_WR);
		dev_dbg(adux1050->dev, "%s - Swap set %x",
			__func__, *swap_state);
	} else {
		dev_dbg(adux1050->dev, "%s - No Swap to be set", __func__);
		*swap_state = ((*swap_state & CLR_NEG_SWAP) & CLR_POS_SWAP);
		err = adux1050->write(adux1050->dev, GET_CONFIG_REG(stg_num),
				swap_state, DEF_WR);
	}
	return err;
}

/**
* \fn void adux1050_force_cal(struct adux1050_chip *adux1050,int cal_time)
* Internal function to perform force calibration of the Chip.
@param adux1050 The chip structure of adux1050 driver
@param cal_time Sleep time required after the force calibration.
@return void
*/
static inline void adux1050_force_cal(struct adux1050_chip *adux1050,
				      int cal_time)
{
	u16 data = 0;

	pr_info("enterint adux1050_force cal.\n");
	adux1050->read(adux1050->dev, BASELINE_CTRL_REG, &data, DEF_WR);
	data = data | FORCE_CAL_MASK;
	adux1050->write(adux1050->dev, BASELINE_CTRL_REG, &data, DEF_WR);
	if (cal_time != 0)
		msleep(cal_time);
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.9*/
	adux1050->high_status = 0;
	if (adux1050->high_thresh_enable)
	{
		adux1050->prev_high_status = adux1050->high_status;
		if (adux1050->send_event)
		{
			send_psensor_uevent(1);
			#ifdef FUNCTION_WHILE_SLEEP
			wake_lock_timeout(&adux1050->adux1050_wakelock, 2 * HZ);
			#endif
		}
        }
	/* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.9*/
}

/**
* \fn inline s16 get_conv_time(struct adux1050_chip *adux1050, int mul_flag)
* To get the required conversion time for the current seting
@adux1050 Chip structure.
@mul_flag Multiplier flag
@return Returns the conversion time required for an updated configurtion
*/
static inline s16 get_conv_time(struct adux1050_chip *adux1050, int mul_flag)
{
	u16 pwr_ctrl_reg = 0;
	u16 cv_time_ctrl = 0;
	u16 pwr_mode = 0;
	u16 stg_num = 0;
	u16 delay_in_ctoc = 0;
	u16 avg = 0;
	u16 osr = 0;
	u16 phase = 0;
	u16 base_time = 0;
	u16 temp_base_time = 0;
	u16 stg_cfg = 0;
	u16 lp_cnt = 0;

	adux1050->read(adux1050->dev, CTRL_REG, &pwr_ctrl_reg, DEF_WR);
	adux1050->read(adux1050->dev, CONV_TIME_CTRL_REG, &cv_time_ctrl,
			DEF_WR);
	pwr_mode = GET_PWR_MODE(pwr_ctrl_reg);
	stg_num = GET_NUM_STG(pwr_ctrl_reg);
	adux1050->tot_stg = stg_num;

	if (pwr_mode == PWR_STAND_BY) {
		return 1;
	} else if (pwr_mode == PWR_FULL_POWER) {
		delay_in_ctoc = ZERO_VAL;
	} else if (pwr_mode == PWR_TIMED_CONV) {
		delay_in_ctoc = GET_TIMED_CONV_TIME(pwr_ctrl_reg);
	} else if (pwr_mode == PWR_AUTO_WAKE) {
		delay_in_ctoc = GET_TIMED_CONV_TIME(pwr_ctrl_reg);
		if (delay_in_ctoc < GET_AUTO_WAKE_TIME(pwr_ctrl_reg))
			delay_in_ctoc = GET_AUTO_WAKE_TIME(pwr_ctrl_reg);
	}
	avg = GET_AVG_CONV(cv_time_ctrl);
	avg = CALC_AVG_CONV(avg); /*AVG based multiplication factor*/
	osr = GET_OSR_CONV(cv_time_ctrl);
	osr = CALC_OSR_CONV(osr); /*OSR multipling factor*/
	phase = GET_CONV_TIME(cv_time_ctrl); /*Phase timing factor*/
	/* Calculate base time as a factor of phase */
	temp_base_time = ((phase + (phase / DECIMAL_BASE)) - 3) / 2;
	temp_base_time *= (avg * osr); /*Set the base_time*/
	for (; lp_cnt < stg_num; lp_cnt++) {
		adux1050->read(adux1050->dev, GET_CONFIG_REG(lp_cnt),
				&stg_cfg, DEF_WR);
		if (IS_NOISE_MEASURE_EN(stg_cfg))
			base_time += (temp_base_time *
				      GET_NOISE_SAMPLE(cv_time_ctrl));
		else
			base_time += temp_base_time;
	}
	if (mul_flag == TWICE_CONV_DELAY_TIME) {
		base_time += base_time; /*Return twice the conv time*/
		base_time += delay_in_ctoc; /*Add the timed conv delay*/
	} else if (mul_flag == CONV_DELAY_TIME) {
		base_time += delay_in_ctoc;
	}
	dev_info(adux1050->dev,	"%s, Basetime(%d), delay_in_ctoc (%d)\n",
		 __func__, base_time, delay_in_ctoc);

	return base_time;
}

/**
* \fn int getstageinfo(struct adux1050_chip *adux1050)
* This function is used to get the current stage information.
@param adux1050 The chip structure of ADUX1050 driver
@return 0 on success.
*/
static int getstageinfo(struct adux1050_chip *adux1050)
{
	u16 temp_reg_val = 0;
	u8 stg_cnt = 0;
	u8 cin_cnt = 0;
	u8 temp_cin;
	adux1050->conn_stg_cnt = 0;

	/* Checking whether Conversion complete interrupt is enabled or not */
	adux1050->read(adux1050->dev, INT_CTRL_REG, &temp_reg_val, DEF_WR);
	adux1050->conv_enable = CHECK_CONV_EN(temp_reg_val);
	ADUX1050_DRIVER_DBG("%s - Checking conv_enable %d temp_reg_val %x\n",
			    __func__, adux1050->conv_enable, temp_reg_val);
	/* Checking whether High threshold interrupt is enabled or not */
	adux1050->high_thresh_enable = CHECK_THRESH_HIGH_EN(temp_reg_val);

	/* Checking whether Low threshold interrupt is enabled or not */
	adux1050->low_thresh_enable = CHECK_THRESH_LOW_EN(temp_reg_val);

	/* How many stages to measure CDC */
	adux1050->read(adux1050->dev, CTRL_REG, &temp_reg_val, DEF_WR);
	adux1050->num_stages = GET_NUM_STG(temp_reg_val);

	/* Find whether stage is connected(either +ve or -ve) or not */
	for (stg_cnt = 0 ; stg_cnt < TOTAL_STG ; stg_cnt++) {
		if (stg_cnt < adux1050->num_stages) {
			adux1050->read(adux1050->dev, GET_CONFIG_REG(stg_cnt),
				       &temp_reg_val, DEF_WR);
			adux1050->stg_info[stg_cnt].status = CIN_NOT_CONNECTED;
			for (cin_cnt = 0; cin_cnt < TOTAL_CIN ; cin_cnt++) {
				temp_cin = (temp_reg_val) & 3;
				if ((temp_cin == CIN_NEG_INPUT) ||
				    (temp_cin == CIN_POS_INPUT)) {
					adux1050->stg_info[stg_cnt].status =
							CIN_CONNECTED;
					adux1050->conn_stg_cnt++;
					dev_info(adux1050->dev,
						 "STG CONNECTED %d,tot=%d\n",
						 stg_cnt,
						 adux1050->conn_stg_cnt);
					break;
				}
				temp_reg_val = temp_reg_val >> 2;
			}
		} else {
			adux1050->stg_info[stg_cnt].status = CIN_NOT_CONNECTED;
			pr_info("%s - CDC not configured for STG_%d\n",
				__func__, stg_cnt);
		}
	}
	return 0;
}

/**
* \fn void update_calib_settings(struct adux1050_chip *adux1050, u16 total_stg,
				u16 *data, bool write_to_reg, u8 file_exist)
* This function updates the calibration output to local register array and
	registers of ADUX1050.
@param adux1050 The chip structure of ADUX1050 driver
@param total_stg Number of stages for which calibration settings to be updated
@param *data Pointer to buffer which contains the calib output
@param write_to_reg A Flag to specify whether to write to registers or not
@param file_exist A Flag to update the calib status
@return 0 on success.
*/
inline void update_calib_settings(struct adux1050_chip *adux1050, u16 total_stg,
				u16 *data, bool write_to_reg, u8 file_exist)
{
	u16 temp_baseline_ctrl = 0;
	u16 int_ctrl_reg = 0;
	u16 stg_cnt = 0;
	u16 stg_num = 0;
	u8 fp = 0;
	u16 cal_base_fail_flag = 0;
	u16 value = 0;
	u16 hys_reg = 0;


	/* Disable the interrupt	*/
	adux1050->read(adux1050->dev, INT_CTRL_REG, &int_ctrl_reg, DEF_WR);
	value = int_ctrl_reg | DISABLE_DEV_INT;
	adux1050->write(adux1050->dev, INT_CTRL_REG, &value, DEF_WR);

	/* Disabling the Auto threhold & Force calib to
	update baseline registers */
	adux1050->read(adux1050->dev, BASELINE_CTRL_REG,
		       &temp_baseline_ctrl, DEF_WR);
	temp_baseline_ctrl = temp_baseline_ctrl & ANTI_FORCE_CAL_MASK;
	if (temp_baseline_ctrl & AUTO_TH_MASK) {
		value = temp_baseline_ctrl;
		value = value & (~AUTO_TH_MASK);
		adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&value, DEF_WR);
	}
	msleep(adux1050->slp_time_conv_complete);

	for (stg_cnt = 0; stg_cnt < total_stg; stg_cnt++) {
		stg_num = data[fp++];
		if ((stg_num >= STG_ZERO) && (stg_num <= STG_THREE)) {

			adux1050->pdata->cal_fact_base[stg_num] = data[fp++];
			adux1050->pdata->cal_offset[stg_num] = data[fp++];
			adux1050->pdata->digi_offset[stg_num] = data[fp++];
			adux1050->pdata->stg_cfg[stg_num] = data[fp++];

			adux1050->bs_reg[stg_num].wr_flag = ADUX1050_ENABLE;
			adux1050->bs_reg[stg_num].value =
				adux1050->pdata->cal_fact_base[stg_num];

			adux1050->reg[GET_OFFSET_REG(stg_num)].wr_flag =
							ADUX1050_ENABLE;
			adux1050->reg[GET_OFFSET_REG(stg_num)].value =
				adux1050->pdata->cal_offset[stg_num];

			adux1050->reg[GET_HYS_REG(stg_num)].wr_flag =
							ADUX1050_ENABLE;
			hys_reg =
			((adux1050->reg[GET_HYS_REG(stg_num)].value) &
			 HYS_BYTE_MASK) |
			 (adux1050->pdata->digi_offset[stg_num] << 8);
			adux1050->reg[GET_HYS_REG(stg_num)].value = hys_reg;

			adux1050->reg[GET_CONFIG_REG(stg_num)].wr_flag =
							ADUX1050_ENABLE;
			adux1050->reg[GET_CONFIG_REG(stg_num)].value =
				adux1050->pdata->stg_cfg[stg_num];
			cal_base_fail_flag |=
				adux1050->pdata->cal_fact_base[stg_num];

			if (write_to_reg == ADUX1050_ENABLE) {
				adux1050->write(
				adux1050->dev, GET_BASE_LINE_REG(stg_cnt),
				&adux1050->pdata->cal_fact_base[stg_cnt],
				DEF_WR);

				adux1050->write(
				adux1050->dev, GET_OFFSET_REG(stg_cnt),
				&adux1050->pdata->cal_offset[stg_cnt], DEF_WR);

				adux1050->read(adux1050->dev,
					       GET_HYS_REG(stg_cnt),
					       &hys_reg, DEF_WR);
				hys_reg =
				((hys_reg & HYS_BYTE_MASK) |
				 (adux1050->pdata->digi_offset[stg_cnt] << 8));
				adux1050->write(adux1050->dev,
						GET_HYS_REG(stg_cnt),
						&hys_reg, DEF_WR);

				adux1050->write(
				adux1050->dev, GET_CONFIG_REG(stg_cnt),
				&adux1050->pdata->stg_cfg[stg_cnt], DEF_WR);
			}

		} else {
			dev_err(adux1050->dev,
				"%s Invalid Stg num in FILP or SYSFS input\n",
				__func__);
			break;
		}
	}

	pr_info("Calib status = %d\n", adux1050->dac_calib.cal_flags);
	/*Restoring the Auto threshold mode if enabled previously*/
	if (temp_baseline_ctrl & AUTO_TH_MASK)
		adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&temp_baseline_ctrl, DEF_WR);
	/* Reenable the interrupt */
	adux1050->write(adux1050->dev, INT_CTRL_REG, &int_ctrl_reg, DEF_WR);

	if (cal_base_fail_flag != 0) {
		adux1050->dac_calib.cal_flags = CAL_RET_SUCCESS;
		/* Getting the stage info */
		getstageinfo(adux1050);
	} else {
		if (file_exist)
			adux1050->dac_calib.cal_flags = CAL_RET_EXIST;
		else
			adux1050->dac_calib.cal_flags = CAL_RET_NONE;
	}
}


/**
* \fn int adux1050_store_register_values(struct adux1050_chip *adux1050)
* This is to retreive the register values from either device tree/local platform
data and store it in local array
@param  adux1050 The Device structure
@return Zero on success
*/
static int adux1050_store_register_values(struct adux1050_chip *adux1050)
{
	u32 lcnt = 0;
	u32 data_cnt = 0;
	const u32 *init_buffer = NULL;
	const __be32 *df_regs = NULL;
	u32 df_prop_length = 0;
#ifdef	CONFIG_OF
	u32 len;
	const __be32 *property = NULL;
#endif
	u8 of_reg_found = false;

#ifdef	CONFIG_OF
	/* Fetching data from Device tree */
	if (adux1050->dt_device_node) {
		df_regs = of_get_property(adux1050->dt_device_node,
					  "adi,adux1050_reg", &df_prop_length);
		/* Fetching required baseline value for STG 0 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg0_base",
					   &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg0_base =
					be32_to_cpu(*property);

			dev_info(adux1050->dev, "valid req_base on %s\n",
				 adux1050->dt_device_node->full_name);
		} else {
			dev_err(adux1050->dev, "Invalid req_base on %s\n",
				adux1050->dt_device_node->full_name);
		}
		/* Fetching required baseline value for STG 1 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg1_base",
					   &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg1_base =
					be32_to_cpu(*property);

			dev_info(adux1050->dev, "valid req_base1 on %s\n",
				 adux1050->dt_device_node->full_name);
		} else {
			dev_err(adux1050->dev, "Invalid req_base1 on %s\n",
				adux1050->dt_device_node->full_name);
		}
		/* Fetching required baseline value for STG 2 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg2_base",
					   &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg2_base =
					be32_to_cpu(*property);

			dev_info(adux1050->dev, "valid req_base2 on %s\n",
				 adux1050->dt_device_node->full_name);
		} else {
			dev_err(adux1050->dev, "Invalid req_base2 on %s\n",
				adux1050->dt_device_node->full_name);
		}
		/* Fetching required baseline value for STG 3 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg3_base", &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg3_base =
					be32_to_cpu(*property);

			dev_info(adux1050->dev, "valid req_base3 on %s\n",
				 adux1050->dt_device_node->full_name);
		} else {
			dev_err(adux1050->dev,	"[ADUX1050]: Invalid req_base3 on %s\n",
				adux1050->dt_device_node->full_name);
		}
	}
#endif
	if ((adux1050->pdata->req_stg0_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg0_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg0_base = HALF_SCALE_VAL;

	if ((adux1050->pdata->req_stg1_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg1_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg1_base = HALF_SCALE_VAL;

	if ((adux1050->pdata->req_stg2_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg2_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg2_base = HALF_SCALE_VAL;

	if ((adux1050->pdata->req_stg3_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg3_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg3_base = HALF_SCALE_VAL;

	/* Data from either DT or initial platform data */
	if ((!df_regs) || (df_prop_length % sizeof(u32))) {
		if (df_prop_length % sizeof(u32))
			dev_err(adux1050->dev, "[ADUX1050]: Malformed prop regs\n");
		init_buffer = adux1050->pdata->init_regs;
		data_cnt = sizeof(adux1050->pdata->init_regs)/sizeof(int);
	} else {
		init_buffer = df_regs;
		data_cnt = df_prop_length / sizeof(u32);
		of_reg_found = true;
	}
	/* Setting enable for INT_CTRL register */
	adux1050->reg[INT_CTRL_REG].wr_flag = ADUX1050_ENABLE;

	for (lcnt = 0; lcnt < data_cnt; lcnt++) {
		u8 addr;
		u16 value;
		/* getting the address and the value to be written */
		if (likely(of_reg_found)) {
			addr = (u8)((be32_to_cpu(init_buffer[lcnt])
						 & ADDR_MASK) >> HEX_BASE);
			value = (u16)(be32_to_cpu(init_buffer[lcnt])
						 & DATA_MASK);
		} else {
			addr = (u8)((init_buffer[lcnt] & ADDR_MASK)
						>> HEX_BASE);
			value = (u16)(init_buffer[lcnt] & DATA_MASK);
		}
		/* Having a copy of device tree values in driver */
		if ((addr >= DEV_ID_REG) &&
		    (addr <= HIGHEST_WR_ACCESS)) {
			adux1050->reg[addr].wr_flag = ADUX1050_ENABLE;
			adux1050->reg[addr].value = value;
			pr_info("!!!!!!! ADDR - %x ; VALUE - %x !!!!\n",
				addr, adux1050->reg[addr].value);
		}
		/* Copy of Baseline registers */
		if ((addr >= BASELINE_STG0_REG) &&
		    (addr <= BASELINE_STG3_REG)) {
			switch (addr) {
			case BASELINE_STG0_REG:
				adux1050->bs_reg[STG_ZERO].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_ZERO].value = value;
				break;
			case BASELINE_STG1_REG:
				adux1050->bs_reg[STG_ONE].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_ONE].value = value;
				break;
			case BASELINE_STG2_REG:
				adux1050->bs_reg[STG_TWO].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_TWO].value = value;
				break;
			case BASELINE_STG3_REG:
				adux1050->bs_reg[STG_THREE].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_THREE].value = value;
			}
		}
	}

	return 0;
}

/**
* \fn  int adux1050_hw_init(struct adux1050_chip *adux1050)
* To initialize the ADUX1050 device with register set defined in
	platform file or device tree
@param  adux1050 The Device structure
@return Zero on success
*/
static int adux1050_hw_init(struct adux1050_chip *adux1050)
{
	u32 lcnt = 0;
	u16 addr;
	u16 slp_time = 0;
	u16 temp_baseline_ctrl = 0;
	u16 pwr_ctrl_buff = 0;
	u16 temp_reg_value = 0;
	u16	value = 0;

	for (lcnt = 0; lcnt < (GLOBAL_REG_CNT + STG_CNF_CNT); lcnt++) {
		addr = lcnt;
		if (adux1050->reg[addr].wr_flag == ADUX1050_ENABLE) {
			value = adux1050->reg[addr].value;
			if (addr == BASELINE_CTRL_REG) {
				value = value & ANTI_FORCE_CAL_MASK;
				if (value & AUTO_TH_MASK) {
					temp_baseline_ctrl = value;
					value = value & (~AUTO_TH_MASK);
				}
			} else if (addr == CTRL_REG) {
				value = value & ~RESET_MASK;
				pwr_ctrl_buff = value;
				value = SET_PWR_MODE(value, PWR_STAND_BY);
			} else if (addr == INT_CTRL_REG) {
				if (adux1050->int_pol == ACTIVE_HIGH)
					adux1050->int_ctrl =
						(value | ACTIVE_HIGH);
				else
					adux1050->int_ctrl =
						(value & ~ACTIVE_HIGH);
				value = adux1050->int_ctrl | DISABLE_DEV_INT;
			}
			ADUX1050_DRIVER_DBG("Addr %x Val %x\n", addr, value);
			adux1050->write(adux1050->dev, addr, &value, DEF_WR);
		}
	}
	/* Baseline registers update */
	if (adux1050->bs_reg[STG_ZERO].wr_flag == ADUX1050_ENABLE)
		adux1050->write(adux1050->dev, BASELINE_STG0_REG,
				&adux1050->bs_reg[STG_ZERO].value,
				DEF_WR);
	if (adux1050->bs_reg[STG_ONE].wr_flag == ADUX1050_ENABLE)
		adux1050->write(adux1050->dev, BASELINE_STG1_REG,
				&adux1050->bs_reg[STG_ONE].value,
				DEF_WR);
	if (adux1050->bs_reg[STG_TWO].wr_flag == ADUX1050_ENABLE)
		adux1050->write(adux1050->dev, BASELINE_STG2_REG,
				&adux1050->bs_reg[STG_TWO].value,
				DEF_WR);
	if (adux1050->bs_reg[STG_THREE].wr_flag == ADUX1050_ENABLE)
		adux1050->write(adux1050->dev, BASELINE_STG3_REG,
				&adux1050->bs_reg[STG_THREE].value,
				DEF_WR);

	/* Restoring the power mode given in configuration */
	if (pwr_ctrl_buff) {
		adux1050->write(adux1050->dev, CTRL_REG,
				&pwr_ctrl_buff, DEF_WR);
		slp_time = get_conv_time(adux1050, CONV_TIME);
		msleep(slp_time);
		ADUX1050_DRIVER_DBG("Addr %x New Val %x\n",
				    CTRL_REG, pwr_ctrl_buff);
	}
	/* Auto threshold enable  */
	if (temp_baseline_ctrl) {
		adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&temp_baseline_ctrl, DEF_WR);
		ADUX1050_DRIVER_DBG("Addr %x New Val %x\n",
				    BASELINE_CTRL_REG,
				    temp_baseline_ctrl);
	}
	/* Clearing the device interrupt STATUS register */
	adux1050->read(adux1050->dev, INT_STATUS_REG, &temp_reg_value, DEF_WR);
	adux1050->prev_low_status = GET_LOW_STATUS(temp_reg_value);
	adux1050->prev_high_status = GET_HIGH_STATUS(temp_reg_value);

	/* Enabling the device interrupt */
	adux1050->write(adux1050->dev, INT_CTRL_REG,
			&adux1050->int_ctrl, DEF_WR);
	ADUX1050_DRIVER_DBG("Addr %x New Val %x\n",
			    INT_CTRL_REG, adux1050->int_ctrl);

	/* Getting the stage info */
	getstageinfo(adux1050);

	/* Storing the sleeping time required for this configuration */
	adux1050->slp_time_conv_complete = get_conv_time(adux1050,
						TWICE_CONV_DELAY_TIME);

	return 0;
}

/**
* \fn offset_write(struct adux1050_chip *adux1050, u16 stg_num, u16 data,
			u16 slp_time)
* Internal function used to write the offset of the Stages
		 with a predeterminded delay
@param adux1050 ADUX chip structure
@param stg_num The stage to which offset has to be written
@param data The Value to be written to the offset register
@param slp_time Sleep time to given after writing the offset register
@return write status is returned
 */
static int offset_write(struct adux1050_chip *adux1050, u16 stg_num, u16 data,
			u16 slp_time)
{
	s32 ret = 0;
	ret = adux1050->write(adux1050->dev, GET_OFFSET_REG(stg_num), &data,
				DEF_WR);
	if (likely(slp_time))
		msleep(slp_time);
	return ret;
}
/**
* \fn inline s16 get_calc_dac_offset(u16 offset, u16 stg_cfg_reg)
* To calculate effective DAC offset value from pos & neg dac offset
@param offset The current offset available.
@param stg_cfg_reg The current stage configuration.
@return the equivalant offset based on the swap bits.
*/
static inline s16 get_calc_dac_offset(u16 offset, u16 stg_cfg_reg)
{
	s16 cal_offset = 0;
	cal_offset = LD_POS_DAC_OFFSET(offset, stg_cfg_reg) +
		     LD_NEG_DAC_OFFSET(offset, stg_cfg_reg);
	return cal_offset;
}

/**
* \fn inline int set_calc_dac_offset(struct adux1050_chip *adux1050,
			      s16 cal_offset, u16 stg_num, u16 *stg_cfg_reg,
			      u16 *offset, u16 sleep_time)
* Helper function to calculate the DAC offset for a stage.
@param adux1050 Chip structure.
@param cal_offset Calculated equalized offset
@param stg_num Stage number
@param *stg_cfg_reg Stage configuration register value.
@param *offset Current offset set.
@param sleep_time Sleep time required for register result cdc.
@return Offset to be set to the register.
*/
static inline int set_calc_dac_offset(struct adux1050_chip *adux1050,
			      s16 cal_offset, u16 stg_num, u16 *stg_cfg_reg,
			      u16 *offset, u16 sleep_time)
{
	u16 err = 0;
	if ((-MAX_OFFSET > cal_offset) || (cal_offset > MAX_OFFSET)) {
		dev_err(adux1050->dev, "[ADUX1050]: %s, offset ERROR(%d)\n",
			__func__, cal_offset);
		return -EINVAL;
	} else {
		*offset = set_dac_offset(cal_offset);
		err = set_swap_state(adux1050, stg_num,
				     stg_cfg_reg, cal_offset);
		if (err < 0)
			return -EIO;
		dev_dbg(adux1050->dev,
			"--->> %s, cal_off(%d)offset(%x) swap(%x)\n",
			__func__, cal_offset, *offset, *stg_cfg_reg);
	}
	return offset_write(adux1050, stg_num, *offset, sleep_time);
}

/**
* \fn int adux1050_offset_check(struct adux1050_chip *adux1050, u16 lp_cnt,
				     u16 max_cnt, s16 dir_flag, u16 slp_time)
* ADUX1050 Calibration routine to check the offset
@param adux1050 The device structure to be calibrated
@param lp_cnt	Loop count
@param max_cnt	Maximum count
@param dir_flag	Direction flag
@param slp_time	Sleep time to be given after offset check
@return The number of configuration data generated
*/
static int adux1050_offset_check(struct adux1050_chip *adux1050, u16 lp_cnt,
				     u16 max_cnt, s16 dir_flag, u16 slp_time)
{
	s16 cal_offset;
	u16 data;
	u16 power_ctrl;
	u16 cin_range;
	u16 lp_count;
	u16 stg_num;
	u16 *offset;
	u16 *stg_cfg_reg;
	adux1050->read(adux1050->dev, CTRL_REG, &power_ctrl, DEF_WR);
	cin_range = GET_CIN_RANGE(power_ctrl);
	adux1050->tot_stg = adux1050->num_stages;
	for (lp_count = 0; lp_count < max_cnt ; lp_count++) {
		for (stg_num = 0; stg_num < adux1050->tot_stg; stg_num++) {
			if ((!CHECK_CAL_STATE(
			    adux1050->dac_calib.sat_comp_stat, stg_num)) ||
			    (adux1050->stg_info[stg_num].status ==
			    CIN_NOT_CONNECTED))
				continue;
			offset = &adux1050->cur_dac_offset[stg_num];
			stg_cfg_reg = &adux1050->cur_swap_state[stg_num];
			adux1050->read(adux1050->dev, GET_RESULT_REG(stg_num),
				       &data, DEF_WR);
			if ((data <= ZERO_SCALE_VALUE) ||
			    (data >= FULL_SCALE_VALUE)) {
				if (lp_count != 0)
					cal_offset = get_calc_dac_offset(
							*offset, *stg_cfg_reg);
				else
					cal_offset = 0;
				cal_offset +=
				(GET_DAC_STEP_SIZE(DAC_CODEOUT_SAT,
						   cin_range) * dir_flag);
				pr_info("[%d]Curr offset set (%d)\n",
					stg_num, cal_offset);
				set_calc_dac_offset(adux1050, cal_offset,
						    stg_num, stg_cfg_reg,
						    offset, 0);
			} else {
				CLR_CAL_STATUS(
				adux1050->dac_calib.sat_comp_stat, stg_num);
				pr_info("%s - SATUR state(%d)\n", __func__,
					adux1050->dac_calib.sat_comp_stat);
				if (!adux1050->dac_calib.sat_comp_stat)
					goto adux_offset_ok;
			}
			ADUX1050_DRIVER_DBG("SATURATION STATUS - %x\n",
					    adux1050->dac_calib.sat_comp_stat);
		}
		msleep(slp_time);
	}
	return -EIO;
adux_offset_ok:
	ADUX1050_DRIVER_DBG("%s, offset ok (%d)\n", __func__, data);
	return 0;
}

static inline void reset_stgcal_flag(struct adux1050_chip *adux1050, u16 *count)
{
	u16 lp_cnt = 0;
	for (; lp_cnt < adux1050->tot_stg; lp_cnt++) {
		if (adux1050->stg_info[lp_cnt].status == CIN_CONNECTED) {
			adux1050->dac_calib.stg_cal_stat |= (1 << lp_cnt);
			count[lp_cnt] = 1;
		}
	}

}

/**
* \fn do_dac_compensation(struct adux1050_chip *adux1050, u16 power_ctrl,
				u16 cin_range, u16 *slp_time)
* Non saturation CDC compensation routine.
@param adux1050 The device structure to be calibrated
@param power_ctrl The power control register read
@param cin_range  The current configuration's CIN range value
@param slp_time	  The sleep time to be given
*/
static int do_dac_compensation(struct adux1050_chip *adux1050, u16 power_ctrl,
				u16 cin_range, u16 *slp_time)
{
	u16 cdc_diff;
	u16 lp_count = 0;
	u16 stg_num = 0;
	u16 count[TOTAL_STG] = {0};
	s16 cal_offset = 0;
	u16 flr_cnt = 0;
	u16 data[TOTAL_STG] = {0};
	u16 hys_reg[TOTAL_STG] = {0};
	u16 offset = 0;
	u16 stg_cfg_reg = 0;
	s16 digi_offset = 0;
	u16 use_digi_offset = 0;
	s32 err = -EIO;
	u16 u16_div = GET_ARB_DAC_STEP_SIZE(cin_range);
	u16 dac_step = u16_div;
	u16 trgt[TOTAL_STG] = {0};
	u16 init_swap_state[TOTAL_STG];
	u16 init_dac_offset[TOTAL_STG];

	struct adux1050_platform_data *pdata = adux1050->pdata;
	*slp_time = get_conv_time(adux1050, TWICE_CONV_DELAY_TIME);
	adux1050->dac_calib.sat_comp_stat = ZERO_VAL;

	/*Writing initial values to zero*/
	for (lp_count = 0; lp_count < adux1050->tot_stg; lp_count++) {
		adux1050->read(adux1050->dev, GET_OFFSET_REG(lp_count),
			       &offset, DEF_WR);
		adux1050->read(adux1050->dev, GET_CONFIG_REG(lp_count),
			       &stg_cfg_reg, DEF_WR);
		init_swap_state[lp_count] = stg_cfg_reg;
		init_dac_offset[lp_count] = offset;
		/*Set initial step count to 1 WARNING: DO NOT set it to zero */
		count[lp_count] = 1;
		cal_offset = get_calc_dac_offset(offset, stg_cfg_reg);
		if (cal_offset != ZERO_VAL)
			cal_offset = 0;
		set_calc_dac_offset(adux1050, cal_offset, lp_count,
				    &stg_cfg_reg, &offset, 0);
		adux1050->cur_swap_state[lp_count] = stg_cfg_reg;
		adux1050->cur_dac_offset[lp_count] = offset;
		trgt[lp_count] = HALF_SCALE_VAL;
		/** Clear the Digital offset if set already*/
		adux1050->read(adux1050->dev, GET_HYS_REG(lp_count),
			       &hys_reg[lp_count], DEF_WR);
		hys_reg[lp_count] &= HYS_BYTE_MASK;
		adux1050->write(adux1050->dev, GET_HYS_REG(lp_count),
				&hys_reg[lp_count], DEF_WR);
		if (adux1050->stg_info[lp_count].status == CIN_CONNECTED) {
			adux1050->dac_calib.sat_comp_stat |= (1 << lp_count);
			adux1050->dac_calib.stg_cal_stat |= (1 << lp_count);
		}
	}
	msleep(*slp_time);
	u16_div = GET_ARB_DAC_STEP_SIZE(GET_CIN_RANGE(power_ctrl));
	dac_step = u16_div;
	/* Clear the device from saturation*/
	flr_cnt = (MAX_OFFSET / (GET_DAC_STEP_SIZE(DAC_CODEOUT_SAT,
					GET_CIN_RANGE(power_ctrl))));
	ADUX1050_DRIVER_DBG("%s - POSITIVE SAT ROUTINE\n", __func__);
	err = adux1050_offset_check(adux1050, lp_count, flr_cnt, 1, *slp_time);
	if (err < 0) {
		ADUX1050_DRIVER_DBG("%s - NEGATIVE SAT ROUTINE\n", __func__);
		err = adux1050_offset_check(adux1050, lp_count, flr_cnt,
					    MINUS_VAL, *slp_time);
		if (err < 0)
			return -EIO;
	}
	/* Set Stage cal status to uncalibrated initial value*/

	/* Make the stages to their corresponding target value*/
	for (lp_count = 0; lp_count < CALIB_LOOP_CNT; lp_count++) {
		u16 prv_data;
		for (stg_num = 0; stg_num < adux1050->tot_stg; stg_num++) {
			/*Skip the stage if already calibrated*/
			if ((!CHECK_CAL_STATE(
			    adux1050->dac_calib.stg_cal_stat, stg_num)) ||
			    (adux1050->stg_info[stg_num].status ==
			    CIN_NOT_CONNECTED))
				continue;
			prv_data = data[stg_num];
			adux1050->read(adux1050->dev, GET_RESULT_REG(stg_num),
				       &data[stg_num], DEF_WR);
			pr_info("DATA %d,OFF %x,SWAP %x,STG %d\n",
				data[stg_num],
				adux1050->cur_dac_offset[stg_num],
				adux1050->cur_swap_state[stg_num],
				stg_num);
			/*Device not in saturation*/
			cdc_diff = abs(data[stg_num] - trgt[stg_num]);
			if (use_digi_offset && (cdc_diff < DIGI_OFFSET_SIZE)) {
				count[stg_num] = 0;
			} else {
				u16_div = abs(prv_data - data[stg_num]);
				u16_div = u16_div / count[stg_num];
				if ((u16_div < (dac_step/2)) ||
				    (u16_div > (dac_step + dac_step)))
					u16_div = dac_step;
				dev_dbg(adux1050->dev, "%s,p_data(%d)",
					__func__, prv_data);
				flr_cnt = cdc_diff / u16_div;
				count[stg_num] = ((cdc_diff % u16_div) >
					 GET_60_PERCENT(u16_div)) ?
					 (flr_cnt + 1) : flr_cnt;
				pr_info("FLC(%d),CNT(%d),DIV(%d),DIF(%d)\n",
					flr_cnt, count[stg_num],
					u16_div, cdc_diff);
			}
			if (count[stg_num] != 0) {
				/* DAC step can be used to minimize the
				 * difference in current and required CDC*/
				cal_offset = get_calc_dac_offset(
					adux1050->cur_dac_offset[stg_num],
					adux1050->cur_swap_state[stg_num]);
				dev_dbg(adux1050->dev, "%s,cal_off %d\n",
					__func__, cal_offset);
				if (data[stg_num] > trgt[stg_num])
					cal_offset += count[stg_num];
				else
					cal_offset -= count[stg_num];
				err = set_calc_dac_offset(adux1050, cal_offset,
				stg_num, &adux1050->cur_swap_state[stg_num],
				&adux1050->cur_dac_offset[stg_num], 0);
				if (err < ZERO_VAL) {
					dev_err(adux1050->dev, "%s S_Off err\n",
						__func__);
					break;
				}
			} else {
				CLR_CAL_STATUS(
				adux1050->dac_calib.stg_cal_stat, stg_num);
				/*When all the stages are in halfscale change to
				  the original target and CIN_range*/
				if ((!adux1050->dac_calib.stg_cal_stat) &&
				    (!use_digi_offset)) {
					trgt[STG_ZERO] = pdata->req_stg0_base;
					trgt[STG_ONE] = pdata->req_stg1_base;
					trgt[STG_TWO] = pdata->req_stg2_base;
					trgt[STG_THREE] = pdata->req_stg3_base;
					pr_info("R TRGT %x,%x,%x,%x\n",
						trgt[STG_ZERO], trgt[STG_ONE],
						trgt[STG_TWO],
						trgt[STG_THREE]);
					power_ctrl = SET_CIN_RANGE(power_ctrl,
								   cin_range);
					adux1050->write(adux1050->dev, CTRL_REG,
							&power_ctrl, DEF_WR);
					u16_div = GET_ARB_DAC_STEP_SIZE(
						    GET_CIN_RANGE(power_ctrl));
					dac_step = u16_div;
					reset_stgcal_flag(adux1050, count);

					use_digi_offset++;
					/* msleep(*slp_time);*/
					break;
				}
				/*Not in saturation and DAC minimal step size
				 * is higher than the required correction*/
				digi_offset = GET_DIGI_OFFSET(trgt[stg_num],
							      data[stg_num]);
				digi_offset = CLAMP_DIGI_OFFSET(digi_offset);

				adux1050->pdata->cal_offset[stg_num] =
					adux1050->cur_dac_offset[stg_num];
				adux1050->pdata->digi_offset[stg_num] =
					(u8)digi_offset;

				hys_reg[stg_num] =
				((hys_reg[stg_num] & HYS_BYTE_MASK) |
				 (adux1050->pdata->digi_offset[stg_num] << 8));
				adux1050->write(adux1050->dev,
						GET_HYS_REG(stg_num),
						&hys_reg[stg_num], DEF_WR);
				ADUX1050_DRIVER_DBG("Hys value = %x\n",
						    hys_reg[stg_num]);
				msleep(*slp_time);
				adux1050->read(adux1050->dev,
					       GET_RESULT_REG(stg_num),
					       &data[stg_num], DEF_WR);
				adux1050->pdata->cal_fact_base[stg_num] =
						data[stg_num];
				adux1050->pdata->stg_cfg[stg_num] =
					adux1050->cur_swap_state[stg_num];
				dev_info(adux1050->dev,
					 "bas(%d)off(%x)st_con(%x)dioff(%d)\n",
					 data[stg_num], offset,
					 stg_cfg_reg, digi_offset);
				err = ZERO_VAL;
				pr_info("CAL STATUS - %x\n",
					adux1050->dac_calib.stg_cal_stat);
				if (!adux1050->dac_calib.stg_cal_stat)
					goto calib_success_break;
				else
					continue;
			}
		}	/*End of stage based loop*/
		msleep(*slp_time);
	}	/*End of for*/
	/*Failed to complete the compensation, return to the original values*/
	for (lp_count = 0; lp_count < adux1050->tot_stg; lp_count++) {
		adux1050->write(adux1050->dev, GET_CONFIG_REG(lp_count),
				&init_swap_state[lp_count], DEF_WR);
		adux1050->write(adux1050->dev, GET_OFFSET_REG(lp_count),
			       &init_dac_offset[lp_count], DEF_WR);
		adux1050->pdata->cal_fact_base[lp_count] = 0;
		adux1050->pdata->cal_offset[lp_count] = 0;
		adux1050->pdata->digi_offset[lp_count] = 0;
		adux1050->pdata->stg_cfg[lp_count] = 0;
		adux1050->dac_calib.cal_flags = CAL_RET_FAIL;
	}
	msleep(*slp_time);
	return err;
calib_success_break:
	adux1050->dac_calib.cal_flags = CAL_RET_SUCCESS;
	return CAL_RET_SUCCESS;
}

#ifdef CONFIG_USE_FILP
/**
* \fn int adux1050_open_calibration(struct adux1050_chip *adux1050,
				bool write_to_reg)
* This routine is used to get the calibration output from the file system and
  update the configuration to the device registers
@param adux1050 The Device Id structure
@param write_to_reg Flag to decide whether to update ADUX1050 registers
@return Returns the size of the read configuration or error number.
 */

static int adux1050_open_calibration(struct adux1050_chip *adux1050,
				bool write_to_reg)
{
	struct file *offset_filp = NULL;
	u16 st_file[FILP_PARAM_CNT * TOTAL_STG + 1] = {0};
	u16 tot_stg_cnt = 0;
	u16 fp = 0;
	s32 err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	offset_filp = filp_open(CAL_DATA_FILE_PATH,
			O_CREAT | O_RDONLY | O_SYNC,
			0666);
	if (IS_ERR(offset_filp)) {
		dev_err(adux1050->dev, "%s: no offset file\n", __func__);
		err = PTR_ERR(offset_filp);
		if (err != -ENOENT)
			dev_err(adux1050->dev, "%s: Can't open calib file\n",
				__func__);
		set_fs(old_fs);
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		return err;
	}


	err = offset_filp->f_op->read(offset_filp, (char *)&st_file,
		(sizeof(u16) * TOTAL_STG * FILP_PARAM_CNT + sizeof(u16)),
		&offset_filp->f_pos);

	if ((err <= 0) || (st_file[0] == 0) ||
	    (st_file[0] > (TOTAL_STG))) {
		dev_err(adux1050->dev, "%s:Can't read, or the file is empty\n",
			__func__);
		err = -EIO;
		adux1050->dac_calib.cal_flags = CAL_RET_EXIST;
		goto flip_exit;
	}
	/* Modified st_file[0] contains only stage number */
	tot_stg_cnt = st_file[fp++];
	dev_info(adux1050->dev, "Total_stg_cnt - %d, st_file[0] - %d\n",
		 tot_stg_cnt, st_file[0]);
	update_calib_settings(adux1050, tot_stg_cnt,
			      &st_file[1], write_to_reg, ADUX1050_ENABLE);

flip_exit:
	/*filp_close(offset_filp, current->files);*/
	filp_close(offset_filp, NULL);
	set_fs(old_fs);

	return err;
}


/**
* \fn int save_calib_val_filp(struct adux1050_chip *adux1050)
* This function is used to save the the calibrated values in a file.
@param adux1050 adux1050_chip structure.
@return zero on success and Negaitive err values.
 */
static int save_calib_val_filp(struct adux1050_chip *adux1050)
{
	s32 err = 0;
	u16 st_file[TOTAL_STG * FILP_PARAM_CNT] = {0};
	struct file *offset_filp = NULL;
	mm_segment_t old_fs;
	u16 fp = 0;
	u16 lp_cnt = 0;

	fp++;
	for (lp_cnt = 0; lp_cnt < adux1050->tot_stg; lp_cnt++) {
		dev_err(adux1050->dev, "LOOP %d ", lp_cnt);
		if (adux1050->stg_info[lp_cnt].status == CIN_CONNECTED) {
			st_file[fp++] = lp_cnt;
			dev_err(adux1050->dev, "{ %x ", lp_cnt);
			st_file[fp++] = adux1050->pdata->cal_fact_base[lp_cnt];
			dev_err(adux1050->dev, "%x ", st_file[fp - 1]);
			st_file[fp++] = adux1050->pdata->cal_offset[lp_cnt];
			dev_err(adux1050->dev, "%x ", st_file[fp - 1]);
			st_file[fp++] =	adux1050->pdata->digi_offset[lp_cnt];
			dev_err(adux1050->dev, "%x ", st_file[fp - 1]);
			st_file[fp++] = adux1050->pdata->stg_cfg[lp_cnt];
			dev_err(adux1050->dev, "%x }\n", st_file[fp - 1]);
		}
	}

	if ((adux1050->dac_calib.cal_flags == CAL_RET_FAIL) ||
	    (adux1050->dac_calib.cal_flags == CAL_RET_NONE))
		adux1050->dac_calib.cal_flags = CAL_RET_EXIST;

	if ((adux1050->dac_calib.action_flag != 0) &&
	    (adux1050->dac_calib.cal_flags == CAL_RET_SUCCESS))
		/*Modified - To stg number only */
		st_file[0] = adux1050->conn_stg_cnt;
	else
		st_file[0] = 0;

	dev_err(adux1050->dev, "{ %x ", st_file[0]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset_filp = filp_open(CAL_DATA_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC,
			0666);
	if (IS_ERR(offset_filp)) {
		dev_err(adux1050->dev, "%s: Can't open file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(offset_filp);
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		goto err_file_open;
	}

	err = offset_filp->f_op->write(offset_filp, (u8 *)&st_file,
			(sizeof(u16) + (sizeof(u16) *
			adux1050->conn_stg_cnt * FILP_PARAM_CNT)),
			&offset_filp->f_pos);
	if (err != ((sizeof(u16) * FILP_PARAM_CNT * adux1050->conn_stg_cnt) +
		    sizeof(u16))) {
		dev_err(adux1050->dev, "%s:Data write failed!!\n", __func__);
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		err = -EIO;
	}

	/*filp_close(offset_filp, current->files);*/
	offset_filp->f_pos = 0;
	filp_close(offset_filp, NULL);


err_file_open:
	set_fs(old_fs);
	return 0;
}
#endif
/**
* \fn inline int adux1050_disable(struct adux1050_chip *adux1050)
* Routine to set the power mode to standby in the ADUX1050 chip
@param adux1050 Chip structure to set the power mode to shutdown
@return Zero on success.
*/
static inline int adux1050_disable(struct adux1050_chip *adux1050)
{
	s32 err;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
	err = adux_power_off(adux1050);
        if (err)
        {
                dev_err(adux1050->dev, "Unable to power off psensor ic!\n");
                adux1050->dev_enable = ADUX1050_ENABLE;
                return err;
        }
        /* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.19*/

	return 0;
}

static inline int adux1050_standby(struct adux1050_chip *adux1050)
{
        u16 data = 0;

        mutex_lock(&adux1050->mutex);
        adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
        if (GET_PWR_MODE(data) != PWR_STAND_BY) {
                data = SET_PWR_MODE(data, PWR_STAND_BY);
                adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);
        }
        mutex_unlock(&adux1050->mutex);

        return 0;
}
/**
* \fn int adux1050_enable(struct adux1050_chip *adux1050)
* Routine to set the driver to enable state in the ADUX1050 chip
@param adux1050 Chip structure to set the power mode to shutdown
@return Zero on success.
*/
static int adux1050_enable(struct adux1050_chip *adux1050)
{

	mutex_lock(&adux1050->mutex);
#ifdef CONFIG_USE_FILP
#ifndef CONFIG_EVAL
	adux1050_open_calibration(adux1050, ADUX1050_DISABLE);
#endif
#endif
	adux1050_hw_init(adux1050);

#ifdef CONFIG_EVAL
	if (adux1050->power_mode_flag == ADUX1050_ENABLE) {
		adux1050->write(adux1050->dev, CTRL_REG,
				&adux1050->ctrl_reg, DEF_WR);
		adux1050->power_mode_flag = ADUX1050_DISABLE;
	}
#endif
	mutex_unlock(&adux1050->mutex);
	return 0;
}

/**
* \fn void adux1050_calibration(struct work_struct *cal_work)
* ADUX1050 Calibration routine
@param cal_work	Pointer to "calib_work", member of ADUX1050 chip structure
@return void
 */
static void adux1050_calibration(struct work_struct *cal_work)
{
	struct adux1050_chip *adux1050 =
		container_of(cal_work, struct adux1050_chip, calib_work);
	u8 slp_time_flag = 0;
	s32 err = 0;
	u16 data;
	u16 power_ctrl;
	u16 lp_cnt;
	u16 slp_time = 0;
	u16 s_t = 0;
	u16 cv_time_ctrl = 0;
	u16 cin_range = 0;
	u16 cur_power = 0;
	u32 start_time = jiffies;

	mutex_lock(&adux1050->mutex);

	adux1050->read(adux1050->dev, INT_CTRL_REG,
		       &adux1050->dac_calib.enable_setting, DEF_WR);
	/*
	 * Disable interrupt and digital offset
	 */
	data = DISABLE_DEV_INT | adux1050->int_pol;
	adux1050->write(adux1050->dev, INT_CTRL_REG, &data, DEF_WR);
	adux1050->dac_calib.cal_flags = CAL_RET_PENDING;

	if (adux1050->dac_calib.action_flag) {
		dev_info(adux1050->dev, "\n\n%s CALIB STARTED\n\n", __func__);
		slp_time = get_conv_time(adux1050, CONV_DELAY_TIME);
		adux1050->read(adux1050->dev, CONV_TIME_CTRL_REG,
			       &cv_time_ctrl, DEF_WR);
		if (CHECK_MIN_TIME(cv_time_ctrl)) {
			data = SET_MIN_TIME(cv_time_ctrl);
			adux1050->write(adux1050->dev, CONV_TIME_CTRL_REG,
					&data, DEF_WR);
			slp_time_flag = 1;
			dev_dbg(adux1050->dev, "%s Check Min time %x\n",
				__func__, data);
		}

		adux1050->read(adux1050->dev, CTRL_REG, &power_ctrl, DEF_WR);
		if (GET_PWR_MODE(power_ctrl) != PWR_FULL_POWER) {
			cur_power = SET_PWR_MODE(power_ctrl, PWR_FULL_POWER);
			adux1050->write(adux1050->dev, CTRL_REG,
					&cur_power, DEF_WR);
			slp_time_flag = slp_time_flag | 0x2;
			slp_time += get_conv_time(adux1050, CONV_TIME);
			dev_dbg(adux1050->dev, "Mode change %d\n", slp_time);
		} else {
			cur_power = power_ctrl;
		}
		cin_range = GET_CIN_RANGE(cur_power);
		if (cin_range != PICO_5) {
			cur_power = SET_CIN_RANGE(cur_power, PICO_5);
			adux1050->write(adux1050->dev, CTRL_REG,
					&cur_power, DEF_WR);
		}
		if (slp_time_flag) {
			dev_dbg(adux1050->dev, "Calib sleep %d\n", slp_time);
			msleep(slp_time);
		}
		err = do_dac_compensation(adux1050, cur_power,
					  cin_range, &s_t);
		dev_dbg(adux1050->dev, "calibration return  %d\n", err);
	} else {
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		for (lp_cnt = 0; lp_cnt < TOTAL_STG; lp_cnt++) {
			adux1050->pdata->cal_fact_base[lp_cnt] = 0;
			adux1050->pdata->cal_offset[lp_cnt] = 0;
			adux1050->pdata->digi_offset[lp_cnt] = 0;
			adux1050->pdata->stg_cfg[lp_cnt] = 0;
		}
	}

#ifdef CONFIG_USE_FILP
	err = save_calib_val_filp(adux1050);
	if (err)
		dev_dbg(adux1050->dev, "calibration save failed [%d]\n", err);
#endif

	#ifdef USE_TRANCEBLITY
	//write_trace(adux1050);
	#endif
	if (adux1050->dac_calib.action_flag) {
		/* If calibration succeed set baseline by using force calib*/
		if (adux1050->dac_calib.cal_flags == CAL_RET_SUCCESS)
			adux1050_force_cal(adux1050, s_t);
		/* Restore the prv conversion time settings */
		if (slp_time_flag & 1) {
			adux1050->write(adux1050->dev, CONV_TIME_CTRL_REG,
					&cv_time_ctrl, DEF_WR);
		}
		/* Restore the previous power Setting */
		if (slp_time_flag & 2) {
			adux1050->write(adux1050->dev, CTRL_REG,
					&power_ctrl, DEF_WR);
		}
	}
	data = adux1050->dac_calib.enable_setting | adux1050->int_pol;
	adux1050->write(adux1050->dev, INT_CTRL_REG, &data, DEF_WR);
	dev_info(adux1050->dev, "\n\nCALIB ENDS Status %d Time %d ms\n\n",
		 adux1050->dac_calib.cal_flags,
		 jiffies_to_msecs(jiffies - start_time));
	mutex_unlock(&adux1050->mutex);

	/*
	if (adux1050->dev_enable == ADUX1050_DISABLE)
		adux1050_disable(adux1050);
	*/

}

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.6*/
static void adux1050_data_monitor_work_fn(struct work_struct *data_monitor_work)
{
	u16 data;
//	u16 val;
//	u16 high_status_change = 0;

	struct delayed_work *dwork = to_delayed_work(data_monitor_work);
	struct adux1050_chip *adux1050 = container_of(dwork, struct adux1050_chip, data_monitor_work);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	mutex_lock(&adux1050->mutex);
        adux1050->read(adux1050->dev, RESULT_STG0_REG, &data, DEF_WR);

	pr_info("data = %d\n", data);
	if ((data > 5000) && (data < 65535))
		adux1050_force_cal(adux1050, get_conv_time(adux1050, CONV_DELAY_TIME));

/*
	adux1050->read(adux1050->dev, RESULT_STG0_REG, &data, DEF_WR);
	adux1050->read(adux1050->dev, BASELINE_STG0_REG, &val, DEF_WR);

	if (adux1050->has_first_calibration == 0)
	{
		adux1050->count++;
		if (adux1050->count >= 10)
		{
			if (data > 9800)
				adux1050_force_cal(adux1050, get_conv_time(adux1050, CONV_DELAY_TIME));
			adux1050->has_first_calibration = 1;
		}
	}
	if ((val > (data + 200)) && (data > 9800))
	{
		adux1050_force_cal(adux1050, get_conv_time(adux1050, CONV_DELAY_TIME));
	}
	else if (data == 0)
	{
		adux1050->high_status = 0;      //lizhi.wu
                if (adux1050->high_thresh_enable)
                {
                        high_status_change = ((adux1050->high_status) ^ (adux1050->prev_high_status));
			adux1050->prev_high_status = adux1050->high_status;
                        if (high_status_change)
                        {
                                if (adux1050->send_event)
                                        indicate_state(adux1050, 1);            //lizhi.wu
				adux1050->read(adux1050->dev, INT_STATUS_REG,
				&adux1050->int_status, DEF_WR);
                        }
                }
	//	indicate_state(adux1050, 1);
	}
	//printk(KERN_CRIT"RESULT_STG0_REG = %d\n", data);
//	schedule_delayed_work(&adux1050->data_monitor_work, 2 * HZ);*/
	mutex_unlock(&adux1050->mutex);
}

#ifdef ENVIRONMENT_CALIBRATION
static void adux1050_env_calib_work_fn(struct work_struct *env_calib_work)
{
	struct delayed_work *dwork = to_delayed_work(env_calib_work);
	struct adux1050_chip *adux1050 = container_of(dwork, struct adux1050_chip, env_calib_work);
	u16 data;
	u8 i;
	u32 calib_sqrt = 0;

//	mutex_lock(&adux1050->mutex);
	if (adux1050->is_env_calib == 0)		//1.5s
	{
		if (adux1050->env_calib_count <= 15)
			adux1050->env_calib_count++;
		else
		{
			adux1050->is_env_calib = 1;
			adux1050->env_calib_count = 0;
			adux1050->result_ave = 0;
		}
		schedule_delayed_work(&adux1050->env_calib_work, msecs_to_jiffies(100));
	}
	else if (adux1050->is_env_calib == 1)
	{
		if (adux1050->env_calib_count < 20)
		{
			adux1050->read(adux1050->dev, RESULT_STG0_REG, &data, DEF_WR);
			adux1050->result_ave += data;
			calib_data_cache[adux1050->env_calib_count] = data;
			//printk(KERN_CRIT"result = %d\n", data);
			//printk(KERN_CRIT"result_ave = %d\n",  adux1050->result_ave);
			adux1050->env_calib_count++;
			schedule_delayed_work(&adux1050->env_calib_work, msecs_to_jiffies(150));
		}
		else
		{
			adux1050->env_calib_count = 0;
			adux1050->is_env_calib = 0;
			adux1050->result_ave /= 20;
			for (i = 0; i < 20; i++)
			{
				if (calib_data_cache[i] >= adux1050->result_ave)
					calib_data_cache[i] -= adux1050->result_ave;
				else
					calib_data_cache[i] = adux1050->result_ave - calib_data_cache[i];
				calib_sqrt += calib_data_cache[i] * calib_data_cache[i];
			}

			//printk(KERN_CRIT"result_ave = %d\n", adux1050->result_ave);
			//printk(KERN_CRIT"calib_sqrt = %d\n", calib_sqrt);
			//printk(KERN_CRIT"result end\n");
			adux1050->read(adux1050->dev, RESULT_STG0_REG, &data, DEF_WR);
			if (calib_sqrt <= 400)
			{
				if ((data > 5000) && (data < 65535))
					adux1050_force_cal(adux1050, get_conv_time(adux1050, CONV_DELAY_TIME));
			}
		}
	}
//	mutex_unlock(&adux1050->mutex);
}
#endif
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.6*/

#ifdef USE_TRANCEBLITY
/*
static void write_trace(struct adux1050_chip *adux1050)
{
	struct file *filp;
	mm_segment_t old_fs;
	loff_t pos = TRANCEBILITY_BASE + 0x1c0;
	int res;
        int ret;
        char buf[TRANCEBILITY_LENGTH] = {0};
	struct adux1050_platform_data *lpdata = adux1050->pdata;

	ret = snprintf(buf, TRANCEBILITY_LENGTH, "0x%04x 0x%04x\n", (u16)lpdata->cal_fact_base[0],
                                ((u16)lpdata->cal_offset[0] | (u16)lpdata->digi_offset[0]));
	filp = filp_open("/dev/block/mmcblk0", O_RDWR, 0);
        if (IS_ERR(filp))
        {
                printk(KERN_CRIT"adux1050 cannot open /dev/block/mmcblk0.\n");
		return;
        }
        printk(KERN_CRIT"%s: filp open success.\n", __func__);

	old_fs = get_fs();
        set_fs(KERNEL_DS);
	res = vfs_write(filp, buf, TRANCEBILITY_LENGTH, &pos);
	set_fs(old_fs);
	if (res < 0)
	{
		printk(KERN_CRIT"%s: write tranceblity failed!\n", __func__);
		filp_close(filp, NULL);
		return;
	}

	printk(KERN_CRIT"%s: write tranceblity success!\n", __func__);
	filp_close(filp, NULL);
}
*/
static int read_trace(struct adux1050_chip *adux1050)
{
	struct file *filp;
	mm_segment_t old_fs;
	loff_t pos = TRANCEBILITY_BASE + 0x1c0;
	int res;
	int ret;
	char buf[TRANCEBILITY_LENGTH] = {0};
	char buf1[7] = {0};
	char buf2[7] = {0};
	char buf3[7] = {0};
	u16 cali_buf[5] = {0};

	filp = filp_open("/dev/block/mmcblk0", O_RDWR, 0);
	if (IS_ERR(filp))
	{
		printk(KERN_CRIT"adux1050 cannot open /dev/block/mmcblk0.\n");
		return -1;
	}
	printk(KERN_CRIT"%s: filp open success.\n", __func__);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	res = vfs_read(filp, buf, TRANCEBILITY_LENGTH, &pos);
	set_fs(old_fs);
	if (res == TRANCEBILITY_LENGTH)
	{
		printk(KERN_CRIT"%s: %s\n", __func__, buf);
		memcpy(buf1, &buf[0], 6);
		buf1[6] = '\0';
		memcpy(buf2, &buf[7], 6);
		buf2[6] = '\0';
		memcpy(buf3, &buf[14], 6);
                buf3[6] = '\0';
		printk(KERN_CRIT"%s: %s\n", __func__, buf1);
		printk(KERN_CRIT"%s: %s\n", __func__, buf2);
		printk(KERN_CRIT"%s: %s\n", __func__, buf3);
		ret = kstrtou16(buf1, 0, &cali_buf[1]);
		if (ret < 0)
		{
			printk(KERN_CRIT"%s: kstrtou16 failed!\n", __func__);
			filp_close(filp, NULL);
			return -1;
		}
		if ((cali_buf[1] > 0x3000) || (cali_buf[1] < 0x2300))
		{
			printk(KERN_CRIT"%s, cali data is error!\n", __func__);
			filp_close(filp, NULL);
                        return -1;
		}
		ret = kstrtou16(buf2, 0, &cali_buf[2]);
                if (ret < 0)
                {
                        printk(KERN_CRIT"%s: kstrtou16 failed!\n", __func__);
			filp_close(filp, NULL);
                        return -1;
                }
		ret = kstrtou16(buf3, 0, &cali_buf[3]);
                if (ret < 0)
                {
                        printk(KERN_CRIT"%s: kstrtou16 failed!\n", __func__);
                        filp_close(filp, NULL);
                        return -1;
                }
		cali_buf[4] = 0x1908;
		printk(KERN_CRIT"%s: 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n", __func__, cali_buf[0], cali_buf[1], cali_buf[2], cali_buf[3], cali_buf[4]);
		update_calib_settings(adux1050, 1, cali_buf, ADUX1050_ENABLE, ZERO_VAL);
	}
	else
	{
		printk(KERN_CRIT"%s: read tranceblity data error\n", __func__);
		filp_close(filp, NULL);
		return -1;
	}
	filp_close(filp, NULL);
	return res;
}

static void trace_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct adux1050_chip *adux1050 = container_of(dwork, struct adux1050_chip, trace_work);

	int ret = 0;
	static int cnt = 0;

	if (cnt < 10)
	{
		ret = read_trace(adux1050);
		if (ret > 0)
		{
			cancel_delayed_work(dwork);
			adux1050_standby(adux1050);
			adux1050->dev_enable = ADUX1050_SUSPEND;
		}
		else
		{
			schedule_delayed_work(dwork, 2 * HZ);
		}
		cnt++;
	}
	else
	{
		cancel_delayed_work(dwork);
		printk(KERN_CRIT"%s: Read tranceblity failed, sensor calibration init failed!\n", __func__);
		adux1050_standby(adux1050);
		adux1050->dev_enable = ADUX1050_SUSPEND;
	}
}
#endif

/**
* \fn static ssize_t store_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
* This Function is used to enable or to disable the device. The Sysfs attribute
is given as "enable", writing a '0' Disables the device.
While writing '1' , enables the device.
@param dev The Device Id structure(linux standard argument)
@param attr Standard Linux Device attributes to the ADUX1050.
@param buf The buffer which contains the data.
@param count The count of bytes to be transfered to the Device.
\note This is evoked upon an echo request in /sys/../<Device> region.
\note This also prints the results in the console for the user.
@return count of data written.
*/
static ssize_t store_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	char *dev_state[2] = { "DISABLED", "ENABLED"};
	s32 err;
	u16 val;
	u16 data = 0;

	err = kstrtou16(buf, 0, &val);

	if (err < 0) {
		dev_err(adux1050->dev, "%s,kstrtoint failed\n", __func__);
		return err;
	}
	if (val > 1) {
		dev_info(adux1050->dev, "%s Invalid- Enable:1 Disable:0\n",
			 __func__);
		return -1;
	}
	dev_info(adux1050->dev, "%s - prv_flag %d curr %d\n", __func__,
		 adux1050->dev_enable, val);
	if (adux1050->dev_enable == val) {
		dev_info(adux1050->dev, "%s - Device already in %s state\n",
			 __func__, dev_state[val]);
		return -1;
	}
	if ((val == ADUX1050_ENABLE) && (adux1050->dev_enable == ADUX1050_SUSPEND))
	{
		enable_irq(gpio_to_irq(psensor_data->irq_gpio));        /* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.24 */
		adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
		if (GET_PWR_MODE(data) != PWR_TIMED_CONV)
		{
			data = SET_PWR_MODE(data, PWR_TIMED_CONV);
			adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);
		}
		adux1050->dev_enable = ADUX1050_ENABLE;
		schedule_delayed_work(&adux1050->data_monitor_work, HZ / 1);
		dev_info(adux1050->dev, "ADUX1050 is enabled\n");
	}
	else if ((val == ADUX1050_DISABLE) && (adux1050->dev_enable == ADUX1050_ENABLE))
	{
		disable_irq(gpio_to_irq(psensor_data->irq_gpio));       /* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.24 */
		adux1050_standby(adux1050);
		adux1050->dev_enable = ADUX1050_SUSPEND;
		if (adux1050->high_thresh_enable)
		{
			adux1050->prev_high_status = adux1050->high_status;
			if (adux1050->send_event)
			{
				send_psensor_uevent(1);
				#ifdef FUNCTION_WHILE_SLEEP
				wake_lock_timeout(&adux1050->adux1050_wakelock, 2 * HZ);
				#endif
			}
		}
		dev_info(adux1050->dev, "ADUX1050 is Disabled\n");
	}
	else
	{
		return -1;
	}
	return count;
}

/**
* \fn static ssize_t show_enable(struct device *dev,
			      struct device_attribute *attr, char *buf)
* This Function is used to show the status of the driver
 Status '1' signifies the device is ENABLED,
 while the status '0' signifies a DISABLED device.
@param dev The Device Id structure(linux standard argument)
@param attr standard Linux Device attributes to the ADUX1050.
@param buf The buffer to store the data to be written.
\note This is evoked upon an cat request in /sys/../<Device> region.
\note This also prints the results in the console for the user.
@return The count of data written.
*/
static ssize_t show_enable(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct adux1050_chip  *adux1050 = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", adux1050->dev_enable);
}

/**
* \fn static ssize_t show_dumpregs(struct device *dev,
			      struct device_attribute *attr, char *buf)
* This Function is used for dumping the registers value of the ADUX1050.
@param dev The Device Id structure(linux standard argument)
@param attr standard Linux Device attributes to the ADUX1050
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device
\note This is evoked upon an cat request in /sys/../<Device> region.
\note This also prints the results in the console for the user.
@return count of data written
*/
static ssize_t show_dumpregs(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 u16temp[MAX_ADUX1050_WR_LEN];
	u32 u32_lpcnt = 0;
	u16 ret = 0;
	mutex_lock(&adux1050->mutex);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
	if (adux1050->dev_enable == ADUX1050_DISABLE)
	{
		dev_err(adux1050->dev, "Device is in disable state\n");
		mutex_unlock(&adux1050->mutex);
		ret = -1;
		return ret;
	}
	/* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/
	dev_info(adux1050->dev, "Global control registers\n");
	adux1050->read(adux1050->dev, DEV_ID_REG, u16temp, GLOBAL_REG_CNT);
	for (; u32_lpcnt < GLOBAL_REG_CNT; u32_lpcnt++) {
		dev_info(adux1050->dev, "Reg 0X%x val 0x%x\n",
			 u32_lpcnt, u16temp[u32_lpcnt]);
		ret += sprintf(buf + ret, "%4x ", u16temp[u32_lpcnt]);
	}
	dev_info(adux1050->dev, "Stage config registers\n");
	adux1050->read(adux1050->dev, CONFIG_STG0_REG, u16temp, STG_CNF_CNT);
	for (u32_lpcnt = CONFIG_STG0_REG;
		u32_lpcnt <= GET_HYS_REG(STG_THREE) ; u32_lpcnt++) {
		dev_info(adux1050->dev, "Reg 0X%x val 0x%x\n",
			 u32_lpcnt, u16temp[u32_lpcnt-CONFIG_STG0_REG]);
		ret += sprintf(buf + ret, "%4x ",
				u16temp[u32_lpcnt-CONFIG_STG0_REG]);
	}
	dev_info(adux1050->dev, "Result/Base/p2p registers\n");
	adux1050->read(adux1050->dev, INT_STATUS_REG, u16temp, STATUS_REG_CNT);
	for (u32_lpcnt = INT_STATUS_REG;
	     u32_lpcnt <= PROX_STATUS_REG; u32_lpcnt++) {
		dev_info(adux1050->dev, "Reg 0X%x val 0x%x\n",
			 u32_lpcnt, u16temp[u32_lpcnt-INT_STATUS_REG]);
		ret += sprintf(buf + ret, "%4x ",
			       u16temp[u32_lpcnt-INT_STATUS_REG]);
	}
	mutex_unlock(&adux1050->mutex);
	return ret;
}

/**
* \fn static ssize_t adux1050_name_show(struct device *dev,
struct device_attribute *attr, char *buf)
* This is used to display the device name of the chipset.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the output buffer with the on/off status
*/
static ssize_t adux1050_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#if 0
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if ((adux1050->dev_enable == ADUX1050_DISABLE) || (adux1050->dev_enable == ADUX1050_SUSPEND))
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
		return snprintf(buf, PAGE_SIZE, "%s\n", DEVICE_NAME);
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/
	adux1050->read(adux1050->dev, INT_STATUS_REG,
		       &adux1050->int_status, DEF_WR);
	return snprintf(buf, PAGE_SIZE, "%s\n", DEVICE_NAME);
#endif

	struct file *cali_filp = NULL;
	mm_segment_t old_fs;
	int ret;
	char temp_buf[40] = {0};
	loff_t pos = 0x8606400 + 0x1c0;
	int i;
	char *ptr = buf;

	cali_filp = filp_open("/dev/block/mmcblk0", O_RDWR, 0);
	if (IS_ERR(cali_filp))
	{
		printk(KERN_CRIT"/dev/block/mmcblk0 file open failed\n");
		return -1;
	}

	old_fs = get_fs();
        set_fs(KERNEL_DS);
	ret = vfs_read(cali_filp, temp_buf, 40, &pos);
	set_fs(old_fs);

	if (ret == 40)
	{
		for (i = 0; i < 40; i++)
		{
			ptr += sprintf(ptr, "0x%02X", temp_buf[i]);
			if ((i % 10) == 9)
				ptr += sprintf(ptr, "\n");
		}
		ptr += sprintf(ptr, "\n");
	}
	else
	{
		printk(KERN_CRIT"file read failed");
		filp_close(cali_filp, NULL);
		return -1;
	}
	filp_close(cali_filp, NULL);
	return (ptr - buf);
}

/**
* \fn static ssize_t adux1050_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
* This is used to display the vendor name of the device.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the output buffer with the on/off status
*/
static ssize_t adux1050_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

/**
* \fn static ssize_t adux1050_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
* This is used to display the Raw CDC data of a stage
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the raw data of all the stages
*/
static ssize_t adux1050_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 temp_cdc = 0;
	s16 ret = 0;

	if (!adux1050->dev_enable) {
		ADUX1050_DRIVER_DBG("Device is not enabled\n");
		goto data_show_err;
	}
	mutex_lock(&adux1050->mutex);
	adux1050->read(adux1050->dev, GET_RESULT_REG(adux1050->stg_raw_cdc),
		       &temp_cdc, DEF_WR);
	ADUX1050_DRIVER_DBG("%s, STG_NO - %d : raw_data - %x\n", __func__,
			    adux1050->stg_raw_cdc, temp_cdc);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x ", temp_cdc);
	mutex_unlock(&adux1050->mutex);

data_show_err:
	return ret;
}

/**
* \fn adux1050_raw_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
* This is used to set the stage number to display CDC raw data.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer which contains the data.
@param count The count of bytes to be transfered to the Device.
\note This is evoked upon an echo request in /sys/../<Device> region.
@return Returns the count of the raw data value of a single stage
*/
static ssize_t adux1050_raw_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;

	err = kstrtou16(buf, 0, &val);
	if (err) {
		dev_err(adux1050->dev, "%s, kstrtoint failed\n", __func__);
		return err;
	}
	if (val < TOTAL_STG)
		adux1050->stg_raw_cdc = val;
	else
		ADUX1050_DRIVER_DBG("%s, Invalid input %d\n", __func__, val);

	return count;
}

/**
* \fn adux1050_send_event_show(struct device *dev,
	struct device_attribute *attr, char *buf)
* This is used to display the send event status of the driver.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the output buffer with the send event status
*/
static ssize_t adux1050_send_event_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", adux1050->send_event);
}

/**
* \fn adux1050_send_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
* This is used to set the send event flag in the driver
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the output buffer
 */
static ssize_t adux1050_send_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 val, err;

	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		dev_err(adux1050->dev, "%s, kstrtoint failed\n", __func__);
		return err;
	}
	if ((val == ADUX1050_ENABLE) || (val == ADUX1050_DISABLE))
		adux1050->send_event = (unsigned char)val;
	else
		ADUX1050_DRIVER_DBG("%s - Invalid input %d\n", __func__, val);
	return count;
}

/**
This is used to show the threshold status of a stage
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the size of the output buffer
*/
static ssize_t adux1050_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 temp_base = 0;
	u16 temp_ht = 0;
	u16 temp_lt = 0;
	s16 ret = 0;

	if (!adux1050->dev_enable) {
		ADUX1050_DRIVER_DBG("Device is not enabled\n");
		goto err;
	}
	mutex_lock(&adux1050->mutex);
	adux1050->read(adux1050->dev,
		       GET_BASE_LINE_REG(adux1050->stg_threshold),
		       &temp_base, DEF_WR);
	adux1050->read(adux1050->dev, GET_HIGH_TH_REG(adux1050->stg_threshold),
		       &temp_ht, DEF_WR);
	adux1050->read(adux1050->dev, GET_LOW_TH_REG(adux1050->stg_threshold),
		       &temp_lt, DEF_WR);
	ADUX1050_DRIVER_DBG("%s, STG_NO - %d : BS: %x, HT - %x : LT - %x\n",
			    __func__, adux1050->stg_threshold,
			    temp_base, temp_ht, temp_lt);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x 0x%04x ",
		       (temp_base + temp_ht), (temp_base - temp_lt));
	mutex_unlock(&adux1050->mutex);

err:
	return ret;
}

/**
This is used to set the stage number to show the threhold details of that stage

@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the raw data value of a single stage
*/
static ssize_t adux1050_theshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;

	err = kstrtou16(buf, 0, &val);
	if (err) {
		dev_err(adux1050->dev, "%s, kstrtoint failed\n", __func__);
		return err;
	}
	if (val < TOTAL_STG)
		adux1050->stg_threshold = val;
	else
		ADUX1050_DRIVER_DBG("%s, Invalid input %d\n", __func__, val);

	return count;
}

/**
This is used to check the DAC offset calibration work status from the driver
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the size of the output buffer
*/
static ssize_t adux1050_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 ret = 0;
	s32 stg_cnt = 0;
	struct adux1050_platform_data *lpdata = adux1050->pdata;

	if (adux1050->dac_calib.cal_flags == CAL_RET_SUCCESS) {
/*
		ret = snprintf(buf + ret, PAGE_SIZE, "%1d %1d ",
			adux1050->dac_calib.cal_flags, adux1050->conn_stg_cnt);
*/
		for (stg_cnt = 0; stg_cnt < adux1050->num_stages; stg_cnt++) {
			if (adux1050->stg_info[stg_cnt].status ==
			    CIN_CONNECTED) {
				/*The STATUS, Stage number, Target, Offset,
					SWAP_state, Digi_offset*/
				dev_info(adux1050->dev,
					 "%1d %1d 0x%04x 0x%04x 0x%04x 0x%04x ",
					 adux1050->dac_calib.cal_flags, stg_cnt,
					 lpdata->cal_fact_base[stg_cnt],
					 lpdata->cal_offset[stg_cnt],
					 lpdata->digi_offset[stg_cnt],
					 lpdata->stg_cfg[stg_cnt]);
				/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.2.28*/
				/*
				ret +=
				snprintf(buf + ret, PAGE_SIZE,
					 "%1d 0x%04x 0x%04x 0x%04x 0x%04x ",
					 (u16)stg_cnt,
					 (u16)lpdata->cal_fact_base[stg_cnt],
					 (u16)lpdata->cal_offset[stg_cnt],
					 (u16)lpdata->digi_offset[stg_cnt],
				 (u16)lpdata->stg_cfg[stg_cnt]);
				*/
				ret +=
                                snprintf(buf + ret, PAGE_SIZE,
                                         "0x%04x 0x%04x 0x%04x 0x%04x\n",
                                         (u16)lpdata->cal_fact_base[stg_cnt],
                                         (u16)lpdata->cal_offset[stg_cnt], (u16)lpdata->digi_offset[stg_cnt],
					 (u16)lpdata->stg_cfg[stg_cnt]);
				/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.2.28*/
			}
		}
	//	ret--;
	} else {
		ret = snprintf(buf, PAGE_SIZE, "%1d\n",
			       (u32)adux1050->dac_calib.cal_flags);
	}
	return ret;
}

/**
This is used to call the DAC offset calibration in the driver
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the input buffer
 */
static ssize_t adux1050_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u32 val;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if ((adux1050->dev_enable == ADUX1050_DISABLE) || (adux1050->dev_enable == ADUX1050_SUSPEND))
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return -1;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/
	err = kstrtoint(buf, 0, &val);

	if (err < 0) {
		dev_err(adux1050->dev, "%s, kstrtoint failed\n", __func__);
		return err;
	}

	if ((val == ADUX1050_ENABLE) || (val == ADUX1050_DISABLE)) {
		/* [PLATFORM]-Removed-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
		/*
		if (adux1050->dev_enable != ADUX1050_ENABLE) {
			mutex_lock(&adux1050->mutex);
			adux1050_hw_init(adux1050);
			mutex_unlock(&adux1050->mutex);
			ADUX1050_DRIVER_DBG("%s, hw_init_done\n", __func__);
		}
		*/
		/* [PLATFORM]-Removed-End by TCTSZ.lizhi.wu, 2015.1.19*/
		adux1050->dac_calib.action_flag = (u8)val;
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		ADUX1050_DRIVER_DBG("%s, Calling schedule_work\n", __func__);
		schedule_work(&adux1050->calib_work);
	} else {
		dev_err(adux1050->dev, "%s, Invalid input %d\n",
			__func__, val);
		return -1;
	}

	return size;
}

/**
Command parsing function for echo/cat commands from command prompt.
This function is called when ever the User tries an echo / cat command
to the /../sysfs/<Device> especially during read/write registers.

@return void Returns Nothing
@see store_reg_read
 */
static int cmd_parsing(const char *buf, u16 *addr, u16 *cnt,
		       u16 *data, u16 data_limit)
{
	char **bp = (char **)&buf;
	u8 *token, minus, parsing_cnt = 0;
	u16 val;
	s32 ret;
	s32 pos;

	data_limit = data_limit + 2;
	while ((token = strsep(bp, SPACE_CHAR))) {
		pos = 0;
		minus = false;
		if ((char)token[pos] == MINUS_CHAR) {
			minus = true;
			pos++;
		}

		ret = kstrtou16(&token[pos], 0, (unsigned short *)&val);
		if (ret)
			return ret;
		if ((parsing_cnt == 0) & (val > HIGHEST_READ_REG))
			return -ERANGE;
		if (minus)
			val *= MINUS_VAL;

		switch (parsing_cnt) {
		case PARSE_ADDR:
			*addr = val;
			break;
		case PARSE_CNT:
			*cnt  = val;
			break;
		default:
		case PARSE_DATA:
			*data = val;
			data++;
			break;
		}
		parsing_cnt++;
		if (parsing_cnt > data_limit)
			return parsing_cnt;
	}
	return parsing_cnt;
}

/**
  This is used to update the calib output to the device from the application space
@return The Size of the Read data
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an wr request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t adux1050_update_calib_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	//u16 data[FILP_PARAM_CNT * TOTAL_STG];
	u16 data[5];
	//u16 status;
	//u16 cnt;

	u8 *token, parsing_cnt = 1;
	char **bp = (char **)&buf;
	s32 pos;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return size;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/
/*
	err = cmd_parsing(buf, &status, &cnt, data,
			  (FILP_PARAM_CNT * TOTAL_STG));
	if (err < 0) {
		dev_err(adux1050->dev, "%s,kstrtos16 failed %x\n",
			__func__, err);
		return err;
	} else {
		dev_err(adux1050->dev, "cmd_ret %d", err);
	}*/
	data[0] = 0x0000;
	while ((token = strsep(bp, SPACE_CHAR)))
	{
		pos = 0;
		err = kstrtou16(&token[pos], 0, (unsigned short *)&data[parsing_cnt]);
		if (err)
			return err;
		parsing_cnt++;
		if (parsing_cnt > 4)
			break;
	}
	printk(KERN_CRIT"0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n", data[0], data[1], data[2], data[3], data[4]);
	if ((data[4] != 0x1908) && (data[4] != 0x3908))
		return size;
	mutex_lock(&adux1050->mutex);
/*
	if (status == 1) {
#ifdef CONFIG_USE_FILP
#ifdef CONFIG_EVAL
		adux1050_open_calibration(adux1050, ADUX1050_ENABLE);
#endif
#else
		dev_err(adux1050->dev, "%s , Invalid option\n", __func__);
#endif

	} else if ((status == CAL_RET_SUCCESS) && (cnt <= TOTAL_STG)) {
			if ((cnt * FILP_PARAM_CNT + 2) != err) {
				dev_err(adux1050->dev, "%s,insuff data(%d)\n",
					__func__, err);
				goto err_data_cnt;
			}
			update_calib_settings(adux1050, cnt, data,
					      ADUX1050_ENABLE, ZERO_VAL);
	} else {
		dev_err(adux1050->dev, "%s, Error for data val(%d)\n",
			__func__, err);
	}
err_data_cnt:
*/
	update_calib_settings(adux1050, 1, data, ADUX1050_ENABLE, ZERO_VAL);
	mutex_unlock(&adux1050->mutex);
	schedule_delayed_work(&adux1050->data_monitor_work, HZ / 2);
	return size;
}



/**
  This is used to show the DAC calibration routine baseline CDC value.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the Size of the output buffer
*/
static ssize_t calib_target_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	struct adux1050_platform_data *pdata = adux1050->pdata;
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d\n", pdata->req_stg0_base,
			pdata->req_stg1_base, pdata->req_stg2_base,
			pdata->req_stg3_base);
}

/**
  This is used to set the DAC calibration offset baseline CDC value.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t calib_target_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 stg_num = 0;
	u16 val = 0;
	s32 err = 0;
	u16 dummy;

	err = cmd_parsing(buf, &stg_num, &val, &dummy, 0);
	if (err < 0) {
		dev_err(adux1050->dev, "%s, kstrtos16 failed\n", __func__);
		return err;
	}
	if ((val <= MAX_CALIB_TARGET) && (val >= MIN_CALIB_TARGET))
		switch (stg_num) {
		case STG_ZERO:
			adux1050->pdata->req_stg0_base = val;
			break;
		case STG_ONE:
			adux1050->pdata->req_stg1_base = val;
			break;
		case STG_TWO:
			adux1050->pdata->req_stg2_base = val;
			break;
		case STG_THREE:
			adux1050->pdata->req_stg3_base = val;
			break;
		default:
		    dev_info(adux1050->dev, "%s,Invalid stg no %d\n",
			     __func__, stg_num);
	} else {
		dev_err(adux1050->dev, "[%d,%d] Limit exceeded(%d)\n",
			MAX_CALIB_TARGET, MIN_CALIB_TARGET, val);
	}

	return size;
}

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.4*/
static ssize_t adux1050_stg0_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t uint_temp[3];

	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return -1;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	adux1050->read(adux1050->dev, BASELINE_STG0_REG, &uint_temp[0], DEF_WR);	//0x79
	adux1050->read(adux1050->dev, HIGH_TH_STG0_REG, &uint_temp[1], DEF_WR);	//0x07
	adux1050->read(adux1050->dev, RESULT_STG0_REG, &uint_temp[2], DEF_WR);	//0x71

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", uint_temp[0], uint_temp[1], uint_temp[2]);
}
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.4*/

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.6*/
static ssize_t adux1050_cin_range_store(struct device *dev, struct device_attribute *attr, char const *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 val;
	s32 err;
	u16 data = 0;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return size;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	err = kstrtou16(buf, 0, &val);
	if (err < 0)
	{
		dev_err(adux1050->dev, "%s, kstrtoint failed\n", __func__);
		return err;
	}

	mutex_lock(&adux1050->mutex);
	if (val == 0)				//+/-5pF
	{
		adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
		data &= ~(0x0003 << 8);
		adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);	//0x01
	}
	else if (val == 1)			//+/-2.5pF
	{
		adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
                data &= ~(0x0003 << 8);
		data |= (0x0001 << 8);
                adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);        //0x01
	}
	else if (val == 2)			//+/-1.25pF
	{
		adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
                data &= ~(0x0003 << 8);
                data |= (0x0002 << 8);
		adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);        //0x01
	}
	else if (val == 3)			//+/-0.625pF
	{
		adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
                data &= ~(0x0003 << 8);
                data |= (0x0003 << 8);
                adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);        //0x01
	}

	mutex_unlock(&adux1050->mutex);
	return size;
}

static ssize_t adux1050_cin_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	u16 data = 0;
	s16 ret = 0;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
		ret = -1;
                return ret;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	mutex_lock(&adux1050->mutex);
	adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
	mutex_unlock(&adux1050->mutex);

	data >>= 8;
	data &= 0x0003;

	switch (data)
	{
		case 0x0000:
			ret = snprintf(buf, PAGE_SIZE, "+/-5pF\n");
			break;
		case 0x0001:
			ret = snprintf(buf, PAGE_SIZE, "+/-2.5pF\n");
			break;
		case 0x0002:
			ret = snprintf(buf, PAGE_SIZE, "+/-1.25pF\n");
			break;
		case 0x0003:
			ret = snprintf(buf, PAGE_SIZE, "+/-0.625pF\n");
			break;
		default:
			break;
	}

	return ret;
}

static ssize_t adux1050_high_threshold_stg0_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	s32 err;
	u16 val;
	u16 temp;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	err = kstrtou16(buf, 0, &val);
	if (err)
	{
		dev_err(adux1050->dev, "%s, kstrtou16 failed\n", __func__);
		return err;
	}

	mutex_lock(&adux1050->mutex);
	temp = val / 48;
	if (temp >= 256)
		temp = 0x00FF;
	adux1050->write(adux1050->dev, HIGH_TH_STG0_REG, &val, DEF_WR);
	adux1050->read(adux1050->dev, HYS_STG0_REG, &val, DEF_WR);
	val &= 0xFF00;
	temp |= val;
	adux1050->write(adux1050->dev, HYS_STG0_REG, &temp, DEF_WR);

	mutex_unlock(&adux1050->mutex);
	return count;
}

static ssize_t adux1050_high_threshold_stg0_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	u16 data;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return -1;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	mutex_lock(&adux1050->mutex);
	adux1050->read(adux1050->dev, HIGH_TH_STG0_REG, &data, DEF_WR);
	mutex_unlock(&adux1050->mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t adux1050_dummy_line_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	u16 val;
	s32 err;
	u16 data = 0;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	err = kstrtou16(buf, 0, &val);
	if (err)
        {
                dev_err(adux1050->dev, "%s, kstrtou16 failed\n", __func__);
                return err;
        }

	mutex_lock(&adux1050->mutex);
	adux1050->read(adux1050->dev, CONFIG_STG0_REG, &data, DEF_WR);

	if (val == 0)				//CIN0 connected to GND
	{
		data = data & (~(0x0003));
	}
	else if (val == 1)			//CIN0 connected to CDC negative input
	{
		data = data & (~(0x0003));
		data |= 0x0001;
	}
	else if (val == 2)			//CIN0 connected to CDC positive input
	{
	//	data = data & (~(0x0003));
          //      data |= 0x0002;
	}
	else if (val == 3)			//CIN0 connected internally to shield
	{
	//	data = data & (~(0x0003));
          //      data |= 0x0003;
	}

	adux1050->write(adux1050->dev, CONFIG_STG0_REG, &data, DEF_WR);

	mutex_unlock(&adux1050->mutex);
	return count;
}

static ssize_t adux1050_dummy_line_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	u16 data;
	s16 ret;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                ret = -1;
                return ret;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	mutex_lock(&adux1050->mutex);
	adux1050->read(adux1050->dev, CONFIG_STG0_REG, &data, DEF_WR);
	mutex_unlock(&adux1050->mutex);

	data &= 0x0003;
	switch (data)
	{
		case 0x0000:
			ret = snprintf(buf, PAGE_SIZE, "Ground\n");
			break;
		case 0x0001:
			ret = snprintf(buf, PAGE_SIZE, "negative\n");
			break;
		case 0x0002:
			ret = snprintf(buf, PAGE_SIZE, "positive\n");
			break;
		case 0x0003:
			ret = snprintf(buf, PAGE_SIZE, "internally\n");
			break;
		default:
			break;
	}

	return ret;
}
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.6*/
#ifdef CONFIG_EVAL

/**
This is used to get register address whose data is to be read and
count of data to be read via sysfs
This function Reads the value at the Device's Register for the i2c client
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
\note This called when the user requires to read the configuration
\note This is evoked upon an echo request in the /sys/.../<Device> region.
\note it hold the register address to be read.
@return The Size of the Read Register 0 if not read
@see cmd_parsing
*/

static ssize_t store_reg_read(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	s32 ret;
	u16 addr;
	u16 cnt = 0;
	u16 val = 0;
	u16 lp_cnt = 0;
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/
	mutex_lock(&adux1050->mutex);
	adux1050->stored_data_cnt = 0;

	ret = cmd_parsing(buf, &addr, &cnt, &val, 0);
	if (ret == -ERANGE || (addr+cnt > HIGHEST_READ_REG)) {
		dev_err(adux1050->dev, "[ADUX1050]: Values not in RANGE\n");
		goto error;
	} else if ((ret == -EINVAL) || (cnt > MAX_ADUX1050_WR_LEN)) {
		dev_err(adux1050->dev, "[ADUX1050]: Invalid COMD/ARG\n");
		goto error;
	} else if (cnt == 0) {
		dev_err(adux1050->dev, "[ADUX1050]: Invalid COM/ARG\n");
		goto error;
	} else {
		val = 0;
	}
	memset(adux1050->stored_reg_data, 0, sizeof(adux1050->stored_reg_data));

	adux1050->read(adux1050->dev, addr, adux1050->stored_reg_data, cnt);
	adux1050->stored_data_cnt = cnt;
	for (lp_cnt = 0; lp_cnt < adux1050->stored_data_cnt; lp_cnt++) {
		ADUX1050_DRIVER_DBG("Reg Read cmd:reg 0x%04x Data 0x%04x\n",
				    addr + lp_cnt,
				    adux1050->stored_reg_data[lp_cnt]);
	}

error:
	mutex_unlock(&adux1050->mutex);
	return count;
}

/**
This is used to read the data of the register address via sysfs sent to reg_read
This functions Reads the value at the Device's Register for the given
client and Prints it in the output window
@param dev The Device ID structure(linux standard argument)
@param attr standard linux device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in the /sys/.../<Device> region.
@return The Size of the read data, 0 if not read

@see dev_get_drvdata
@see store_reg_read
*/

static ssize_t show_reg_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	s32 val = 0, lp_cnt = 0;
	struct adux1050_chip  *adux1050 = dev_get_drvdata(dev);
	for (lp_cnt = 0; lp_cnt < adux1050->stored_data_cnt; lp_cnt++) {
		val += snprintf(buf + val, PAGE_SIZE, "0x%x ",
				adux1050->stored_reg_data[lp_cnt]);
	}
	return val;
}

/**
This is used to write data to a register through i2c.
This functions Writes the value of the buffer to the given client
provided the count value to write
@param dev The device ID structure(linux standard argument)
@param attr standard linux device attributes to the ADUX1050
@param count The number of bytes to write from the buffer
@param buf The buffer to store the Read data
\note This is used to store the register address to write the data.
\note This is evoked upon an echo request in the /sys/.../<Device> region.
\note This also prints the command received before writing the Registers.
@return The Size of the writen Data, 0 if not writen
*/

static ssize_t store_reg_write(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 addr, cnt;
	u16 wr_data[MAX_ADUX1050_WR_LEN], loop_cnt;
	s32 ret;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	mutex_lock(&adux1050->mutex);

	ret = cmd_parsing(buf, &addr, &cnt, &wr_data[0], MAX_ADUX1050_WR_LEN);
	if (ret == -ERANGE || (ret == -EINVAL)) {
		dev_err(adux1050->dev, "%s - Values not in RANGE\n", __func__);
		goto error;
	} else if ((addr >= BASELINE_STG0_REG &&
		    addr <= GET_BASE_LINE_REG(STG_THREE)) &&
		   (addr+cnt <= GET_BASE_LINE_REG(STG_THREE) + 1)) {
		ADUX1050_DRIVER_DBG("%s Baseline write register\n", __func__);
	} else if ((addr + cnt) > (HIGHEST_WR_ACCESS + 1)) {
		dev_err(adux1050->dev, "%s - Addr reaches RD only regs\n",
			__func__);
		goto error;
	} else if (cnt > MAX_ADUX1050_WR_LEN) {
		dev_err(adux1050->dev, "%s - Max write length\n", __func__);
		goto error;
	}
	ADUX1050_DRIVER_DBG("Register Write command :reg= 0x%x, size= %d\n",
			    addr, cnt);

	for (loop_cnt = 0; loop_cnt < cnt; loop_cnt++) {

		/* Storing local of the values before writing to registers */
		adux1050->reg[addr+loop_cnt].wr_flag = ADUX1050_ENABLE;
		adux1050->reg[addr+loop_cnt].value = wr_data[loop_cnt];

		/* Conditions to be checked
		   1.SW RESET, FORCE CALIB, AUTO_Threshold, power state ... */
		if ((addr + loop_cnt) == CTRL_REG) {
			/* Clearing SW Reset bit */
			if (CHK_SW_RESET_EN(wr_data[loop_cnt])) {
				ADUX1050_DRIVER_DBG("S/W reset not allowed");
				ADUX1050_DRIVER_DBG("- Use reset sysfs!!!\n");
				wr_data[loop_cnt] =
					CLR_SW_RESET_EN(wr_data[loop_cnt]);
			}
			/*Device is not allowed to wakeup
			  during drv disable state*/
			if (adux1050->dev_enable == ADUX1050_DISABLE) {
				if (GET_PWR_MODE(wr_data[loop_cnt]) !=
				    PWR_STAND_BY) {
					/* PWR saved, used in next enable*/
					adux1050->ctrl_reg = wr_data[loop_cnt];
					adux1050->power_mode_flag =
							ADUX1050_ENABLE;
					wr_data[loop_cnt] =
				SET_PWR_MODE(wr_data[loop_cnt], PWR_STAND_BY);
				}
			}
		} else if (addr + loop_cnt == BASELINE_CTRL_REG) {
			if (CHK_FORCE_CALIB_EN(wr_data[loop_cnt])) {
				ADUX1050_DRIVER_DBG("FORCE CAL not allowed-");
				ADUX1050_DRIVER_DBG("Use force calib sysfs\n");
				wr_data[loop_cnt] =
					CLR_FORCE_CALIB_EN(wr_data[loop_cnt]);
			}
		}
		ADUX1050_DRIVER_DBG("DATA = 0x%04X\n", wr_data[loop_cnt]);
	}
	adux1050->write(adux1050->dev, addr, wr_data, cnt);
error:
	mutex_unlock(&adux1050->mutex);
	return count;
}

/**
This is used to update the stage details upon every change in configuration
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_update_config(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;
	err = kstrtou16(buf, 0, &val);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	if (err < 0) {
		dev_err(adux1050->dev, "%s, kstrtos16 failed\n", __func__);
		return err;
	}
	if (val == 1) {
			mutex_lock(&adux1050->mutex);
			getstageinfo(adux1050);
			mutex_unlock(&adux1050->mutex);
			msleep(adux1050->slp_time_conv_complete);
			adux1050->slp_time_conv_complete =
				get_conv_time(adux1050, TWICE_CONV_DELAY_TIME);
	} else {
		ADUX1050_DRIVER_DBG("Invalid Input %d\n", val);
	}

	return count;
}

/**
This is used to do Software reset of the device of ADUX1050
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_sw_reset(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 ctrl_reg_val;
	s32 err;
	u16 val;
	err = kstrtou16(buf, 0, &val);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	if (err < 0) {
		dev_err(adux1050->dev, "%s - kstrtos16 failed\n", __func__);
		return err;
	}
	if (val == 1) {
		mutex_lock(&adux1050->mutex);
		/* SW reset is enabled in ctrl register */
		adux1050->read(adux1050->dev, CTRL_REG, &ctrl_reg_val, DEF_WR);
		ctrl_reg_val = SET_SW_RESET_EN(ctrl_reg_val);
		adux1050->write(adux1050->dev, CTRL_REG,
				&ctrl_reg_val, DEF_WR);
		msleep(200);
		/*Device is put to disable mode */
		if (ADUX1050_ENABLE == adux1050->dev_enable) {
			adux1050->dev_enable = ADUX1050_DISABLE;
			disable_irq_wake(psensor_data->irq_gpio);
			disable_irq(psensor_data->irq_gpio);
		}
		mutex_unlock(&adux1050->mutex);
	} else {
		ADUX1050_DRIVER_DBG("%s - Invalid input %d\n", __func__, val);
	}
	return count;
}

/**
This is used to do factory calibration of the ADUX1050
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_force_calib(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 baseline_ctrl_val;
	s32 err;
	u16 val;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.19*/
        if ((adux1050->dev_enable == ADUX1050_DISABLE) || (adux1050->dev_enable == ADUX1050_SUSPEND))
        {
                dev_err(adux1050->dev, "Device is in disable state\n");
                return count;
        }
        /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.19*/

	err = kstrtou16(buf, 0, &val);

	if (err < 0) {
		dev_err(adux1050->dev, "%s, kstrtos16 failed\n", __func__);
		return err;
	}
	if (val == 1) {
		/* Force Calibration is enabled in Baseline ctrl register */
		/*TODO: Check AUTO-TH is enable or not */
		/*TODO: Can the Force calibratin be done
		   when the device is at DISABLED state?*/
		mutex_lock(&adux1050->mutex);
		adux1050->read(adux1050->dev, BASELINE_CTRL_REG,
			       &baseline_ctrl_val, DEF_WR);
		baseline_ctrl_val = SET_FORCE_CALIB_EN(baseline_ctrl_val);
		adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&baseline_ctrl_val, DEF_WR);
		msleep(300);
		mutex_unlock(&adux1050->mutex);
		adux1050->high_status = 0;	//lizhi.wu
		if (adux1050->high_thresh_enable)
		{
			adux1050->prev_high_status = adux1050->high_status;
			if (adux1050->send_event)
			{
				send_psensor_uevent(1);
				#ifdef FUNCTION_WHILE_SLEEP
				wake_lock_timeout(&adux1050->adux1050_wakelock, 2 * HZ);
				#endif
			}
		}
	} else {
		ADUX1050_DRIVER_DBG("%s,Value is invalid\n", __func__);
	}
	return count;
}

#endif

/*
The sysfs attributes used in the driver follows
*/
/*--------------------------------------------------------------*/
static DEVICE_ATTR(adux1050_enable, S_IRUGO | S_IWUSR | S_IWGRP,	//modified by lizhi.wu 2015-1-6 for test
	show_enable, store_enable);
/*--------------------------------------------------------------*/
static struct device_attribute dev_attr_sensor_name =
	__ATTR(adux1050_device_name, S_IRUSR | S_IRGRP,
	       adux1050_name_show, NULL);
static struct device_attribute dev_attr_sensor_vendor =
	__ATTR(adux1050_vendor, S_IRUSR | S_IRGRP,
	       adux1050_vendor_show, NULL);
static struct device_attribute dev_attr_sensor_raw_data =
	__ATTR(adux1050_raw_data, S_IRUGO | S_IWUSR | S_IWGRP,
	       adux1050_raw_data_show, adux1050_raw_data_store);
static struct device_attribute dev_attr_sensor_send_event =
	__ATTR(adux1050_send_event, S_IRUGO | S_IWUSR | S_IWGRP,
	       adux1050_send_event_show, adux1050_send_event_store);
static struct device_attribute dev_attr_sensor_threshold =
	__ATTR(adux1050_threshold, S_IRUGO | S_IWUSR | S_IWGRP,
	       adux1050_threshold_show, adux1050_theshold_store);
static struct device_attribute dev_attr_sensor_calibration =
	__ATTR(adux1050_calibration, S_IRUGO | S_IWUSR | S_IWGRP,		//modified by lizhi.wu 2015-1-6 for test
	       adux1050_calibration_show, adux1050_calibration_store);
static struct device_attribute dev_attr_sensor_update_calib =
	__ATTR(adux1050_update_calib, S_IRUGO | S_IWUSR | S_IWGRP,
	       NULL, adux1050_update_calib_store);
static struct device_attribute dev_attr_sensor_dump =
	__ATTR(adux1050_status, S_IRUSR | S_IRGRP,
	       show_dumpregs, NULL);
static struct device_attribute dev_attr_sensor_calib_target =
	__ATTR(adux1050_calib_target, S_IRUGO | S_IWUSR | S_IWGRP,
	       calib_target_show, calib_target_store);
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.1.4*/
static struct device_attribute dev_attr_adux1050_stg0_result =
	__ATTR(adux1050_stg0_result, S_IRUGO | S_IWUSR | S_IWGRP,
		adux1050_stg0_result_show, NULL);
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.4*/
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.6*/
static struct device_attribute dev_attr_adux1050_cin_range =
	__ATTR(adux1050_cin_range, S_IRUGO | S_IWUSR | S_IWGRP,
		adux1050_cin_range_show, adux1050_cin_range_store);
static struct device_attribute dev_attr_adux1050_high_threshold_stg0 =
	__ATTR(adux1050_high_threshold_stg0, S_IRUGO | S_IWUSR | S_IWGRP,
		adux1050_high_threshold_stg0_show, adux1050_high_threshold_stg0_store);
static struct device_attribute dev_attr_adux1050_dummy_line =
        __ATTR(adux1050_dummy_line, S_IRUGO | S_IWUSR | S_IWGRP,
               adux1050_dummy_line_show, adux1050_dummy_line_store);
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.6*/
#ifdef CONFIG_EVAL
static struct device_attribute dev_attr_sensor_reg_read =
	__ATTR(adux1050_reg_read, S_IRUGO | S_IWUSR | S_IWGRP,
	       show_reg_read, store_reg_read);
static struct device_attribute dev_attr_sensor_reg_write =
	__ATTR(adux1050_reg_write, S_IRUGO | S_IWUSR | S_IWGRP,
	       NULL, store_reg_write);
static struct device_attribute dev_attr_sensor_update_config =
	__ATTR(adux1050_update_config, S_IRUGO | S_IWUSR | S_IWGRP,
	       NULL, store_update_config);
static struct device_attribute dev_attr_sensor_sw_reset =
	__ATTR(adux1050_sw_reset, S_IRUGO | S_IWUSR | S_IWGRP,
	       NULL, store_sw_reset);
static struct device_attribute dev_attr_sensor_force_calib =
	__ATTR(adux1050_force_calib, S_IRUGO | S_IWUSR | S_IWGRP,		//modified by lizhi.wu 2015-1-6 for test
	       NULL, store_force_calib);
#endif

static struct attribute *adux1050_attrs[] = {
	&dev_attr_adux1050_enable.attr,
	&dev_attr_sensor_name.attr,
	&dev_attr_sensor_vendor.attr,
	&dev_attr_sensor_raw_data.attr,
	&dev_attr_sensor_send_event.attr,
	&dev_attr_sensor_threshold.attr,
	&dev_attr_sensor_calibration.attr,
	&dev_attr_sensor_update_calib.attr,
	&dev_attr_sensor_dump.attr,
	&dev_attr_sensor_calib_target.attr,
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.4*/
	&dev_attr_adux1050_stg0_result.attr,
	&dev_attr_adux1050_cin_range.attr,
	&dev_attr_adux1050_high_threshold_stg0.attr,
	&dev_attr_adux1050_dummy_line.attr,
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.4*/
#ifdef CONFIG_EVAL
	&dev_attr_sensor_reg_read.attr,
	&dev_attr_sensor_reg_write.attr,
	&dev_attr_sensor_update_config.attr,
	&dev_attr_sensor_sw_reset.attr,
	&dev_attr_sensor_force_calib.attr,
#endif
	NULL,
};

static struct attribute_group adux1050_attr_group = {
	.name = NULL,
	.attrs = adux1050_attrs,
};

/**
This Routine reads the Device ID to confirm the existance
of the Device in the System.
@param  adux1050 The Device structure
@return 0 on Successful detection of the device,-ENODEV on err.
*/
static int adux1050_hw_detect(struct adux1050_chip *adux1050)
{
	u16 data;

	adux1050->read(adux1050->dev, DEV_ID_REG, &data, DEF_WR);
	if (likely((data & ADUX1050_ID_MASK) == ADUX1050_GENERIC_ID)) {
		adux1050->product = ADUX1050_GENERIC_ID >> 8;
		adux1050->version = data & REV_ID_MASK >> 4;
		adux1050->metal_id = data & METAL_REV_ID_MASK;
		dev_info(adux1050->dev, "Found ADUX1050, rev:%x met_id:%x\n",
			 adux1050->version, adux1050->metal_id);
		return 0;
	} else {
		dev_err(adux1050->dev, "ADUX1050 Not Found,ID %4x\n", data);
		return -ENODEV;
	}
}

/**
Threaded IRQ Handler -- Assigns interrupt handling to work
@param handle The data of the ADUX1050 Device
@param irq The Interrupt Request queue to be assigned for the device.
@return IRQ_HANDLED
*/
static irqreturn_t adux1050_isr_thread(int irq, void *handle)
{
	struct adux1050_chip *adux1050 = handle;
	mutex_lock(&adux1050->mutex);

	if (!work_pending(&adux1050->work))
		schedule_work(&adux1050->work);
	else
		/*Cleared the interrupt for future intterupts to occur*/
		adux1050->read(adux1050->dev, INT_STATUS_REG,
				&adux1050->int_status, DEF_WR);

	mutex_unlock(&adux1050->mutex);
	return IRQ_HANDLED;
}
/**
conv_complete_cdc_fetch - Fetch the CDC for the connected stage after
receving the conversion sequence complete interrupt is asserted
@param	adux1050	The chip structure of ADUX1050 chip
@return void
*/
static inline void conv_complete_cdc_fetch(struct adux1050_chip *adux1050)
{
	u8 stg_cnt = 0;
	u16 result_cdc;
	unsigned int event_value = 0;

	for (stg_cnt = 0; stg_cnt < adux1050->num_stages; stg_cnt++) {
		/** Fetch the CDC only if that stage is connected */
		if (adux1050->stg_info[stg_cnt].status == CIN_CONNECTED) {
			adux1050->read(adux1050->dev, GET_RESULT_REG(stg_cnt),
				       &result_cdc, DEF_WR);
			pr_info( "STG_NO %x - CDC %x\n", stg_cnt, result_cdc);
			if (adux1050->send_event) {
				event_value = PACK_FOR_CDC_EVENT(result_cdc,
								 stg_cnt);
			 
				//input_event(adux1050->input, EV_MSC,			/* [PLATFORM]-Removed by TCTSZ.lizhi.wu, 2014.12.30 */ 
				//	    MSC_RAW, event_value);
				//input_sync(adux1050->input);
			}
		}
	}
}

/**
This function is used to send ACTIVE event for a stage in ADUX1050
@param adux1050 The ADUX1050 chip structure pointer
@param stg_num	The stage nmmber for which the event to be sent
@param Threshold type	Indicates Low or High threshold
@return void
*/

/* [PLATFORM]-Removed-BEGIN by TCTSZ.lizhi.wu, 2014.12.30 */
/*
static inline void indicate_active_state(struct adux1050_chip *adux1050,
					 int stg_num, int threshold_type)
{
	unsigned int event_value = 0;

	ADUX1050_DRIVER_DBG("%s\n", __func__);
	event_value = PACK_FOR_ACTIVE_STATE(stg_num, threshold_type);
	input_event(adux1050->input, EV_MSC, MSC_RAW, event_value);
	input_sync(adux1050->input);

}*/
/* [PLATFORM]-Removed End by TCTSZ.lizhi.wu, 2014.12.30 */

/**
This function is used to send IDLE event for a stage in ADUX1050
@param adux1050 The ADUX1050 chip structure pointer
@param stg_num	The stage nmmber for which the event to be sent
@param Threshold type	Indicates Low or High threshold
@return void
*/

/* [PLATFORM]-Removed-BEGIN by TCTSZ.lizhi.wu, 2014.12.30 */
/*
static inline void indicate_idle_state(struct adux1050_chip *adux1050,
				       int stg_num, int threshold_type)
{
	unsigned int event_value = 0;

	ADUX1050_DRIVER_DBG("%s\n", __func__);
	event_value = PACK_FOR_IDLE_STATE(stg_num, threshold_type);
	input_event(adux1050->input, EV_MSC, MSC_RAW, event_value);
	input_sync(adux1050->input);
}*/
/* [PLATFORM]-Removed End by TCTSZ.lizhi.wu, 2014.12.30 */

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.12.30 */
#if 0
static inline void indicate_state(struct adux1050_chip *adux1050, int val)
{
	char *penv[2];
	char psensor[20];

	psensor_data->psensor_dev->state = val;
	snprintf(psensor, sizeof(psensor), "SWITCH_STATE=%d", val);
	penv[0] = psensor;
	penv[1] = NULL;

	if (val == 0)		//near
	{
		kobject_uevent_env(&psensor_data->psensor_dev->dev->kobj, KOBJ_ADD, penv);
	}
	else if (val == 1)	//far
	{
		kobject_uevent_env(&psensor_data->psensor_dev->dev->kobj, KOBJ_REMOVE, penv);
	}
}
#endif
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2014.12.30 */

/**
high_threshold_int_check - Identify which stage asserts the high threshold
interrupt.After identifying the stage, the state(ACTIVE,IDLE) of the stage
is sent as input event
@param	adux1050 The chip structure of ADUX1050 chip
@param	high_status_change Contains stage number which asserts INT.
@return void
*/
static inline void high_threshold_int_check(struct adux1050_chip *adux1050,
					    u16 high_status_change)
{
	u8 stg_cnt;
	u8 temp_ht_enable = adux1050->high_thresh_enable;

	printk("<2>""-------%s,%d\n",__func__,__LINE__);

	for (stg_cnt = 0; stg_cnt < TOTAL_STG; stg_cnt++) {
		if (temp_ht_enable & 1) {
			if (high_status_change & 1) {
				if (adux1050->high_status & 1) {
					dev_info(adux1050->dev, "St = %de\n",
						 stg_cnt);
					if (adux1050->send_event)
					{
						//indicate_active_state(adux1050, stg_cnt, TH_HIGH);	/* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.30 */
						send_psensor_uevent(0);
						#ifdef FUNCTION_WHILE_SLEEP
						wake_lock_timeout(&adux1050->adux1050_wakelock, 2 * HZ);
						#endif
						#ifdef ENVIRONMENT_CALIBRATION
						adux1050->is_env_calib = 0;
                                                adux1050->env_calib_count = 0;
						schedule_delayed_work(&adux1050->env_calib_work, msecs_to_jiffies(100));
						#endif
					}
				} else {
					dev_info(adux1050->dev, "St = %dx\n",
						 stg_cnt);
					if (adux1050->send_event)
					{
						//indicate_idle_state(adux1050, stg_cnt, TH_HIGH);
						send_psensor_uevent(1);		/* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.30 */
						#ifdef FUNCTION_WHILE_SLEEP
						wake_lock_timeout(&adux1050->adux1050_wakelock, 2 * HZ);
						#endif
						#ifdef ENVIRONMENT_CALIBRATION
						cancel_delayed_work_sync(&adux1050->env_calib_work);
						adux1050->is_env_calib = 0;
						adux1050->env_calib_count = 0;
						#endif
					 }
				}
			}
		}
		temp_ht_enable = temp_ht_enable >> 1;
		high_status_change = high_status_change >> 1;
		adux1050->high_status = adux1050->high_status >> 1;
	}
}

/**
Low_threshold_int_check - Identify which stage asserts the low threshold
interrupt.After identifying the stage, the state(ACTIVE,IDLE) of the stage
is sent as input event
@param	adux1050 The chip structure of ADUX1050 chip
@param	low_status_change Contains stage number which asserts INT.
@return void
*/
/* [PLATFORM]-Removed Begin by TCTSZ.lizhi.wu, 2015.1.9*/
/*
static inline void low_threshold_int_check(struct adux1050_chip *adux1050,
					   u16 low_status_change)
{
	u8 stg_cnt;
	u8 temp_lt_enable = adux1050->low_thresh_enable;
	u16 data;

	printk("<2>""-------%s,%d\n",__func__,__LINE__);

	for (stg_cnt = 0; stg_cnt < TOTAL_STG; stg_cnt++)
	{
		//adux1050->read(adux1050->dev, RESULT_STG0_REG, &data, DEF_WR);
			if (low_status_change & 1) {
				if (adux1050->low_status & 1) {
					dev_info(adux1050->dev, "St = %de\n",
						 stg_cnt);
					if (adux1050->send_event)
					{
					}
						//indicate_active_state(adux1050, stg_cnt, TH_LOW);
						//indicate_state(adux1050, 0);		//[PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.30
				} else {
					dev_info(adux1050->dev, "St = %dx\n",
						 stg_cnt);
					if (adux1050->send_event)
					{
					}
						//indicate_idle_state(adux1050, stg_cnt, TH_LOW);
						//indicate_state(adux1050, 1);		//[PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.30
				}
			}
			temp_lt_enable = temp_lt_enable >> 1;
			low_status_change = low_status_change >> 1;
			adux1050->low_status = adux1050->low_status >> 1;
		}
}
*/
/* [PLATFORM]-Removed End by TCTSZ.lizhi.wu, 2015.1.9*/

void usb_calibration_work_func(struct work_struct *work)
{
	struct adux1050_chip *adux1050 = NULL;

        if ((p_sendevent == NULL) && (psensor_power_on == 1))
        {
                printk(KERN_CRIT"%s: usb pull in or out, psensor is 128?\n", __func__);
                if (gpio_is_valid(psensor_data->enable_gpio))
                        gpio_direction_output(psensor_data->enable_gpio, 0);
                else
                        return;
                msleep(100);
                if (gpio_is_valid(psensor_data->enable_gpio))
                        gpio_direction_output(psensor_data->enable_gpio,1);
		printk(KERN_CRIT"%s: IQS128 reset!\n", __func__);
                return;
        }
        else if ((p_sendevent == NULL) && (psensor_power_on == 0))
                return ;

        adux1050 = container_of(p_sendevent, struct adux1050_chip, send_event);
        printk(KERN_CRIT"%s:usb pull in or out, adux1050 calibration!\n", __func__);
        if (adux1050->dev_enable == ADUX1050_ENABLE)
                schedule_delayed_work(&adux1050->data_monitor_work, 1 * HZ);
}

void adux1050_usb_cali(void)
{
        schedule_work(&psensor_data->usb_cali_work);
}

/**
Interrupt work Handler -- Handles the interrupts from ADUX1050
@param work The work structure for the ADUX1050 chip
@return void Nothing returned
*/
static void adux1050_isr_work_fn(struct work_struct *work)
{
	struct adux1050_chip *adux1050 =  container_of(work,
						   struct adux1050_chip, work);
	u16 high_status_change = 0;
	//u16 low_status_change = 0;		/* [PLATFORM]-Removed by TCTSZ.lizhi.wu, 2015.1.9*/
	u16 data;

	printk("<2>""-------%s,%d\n",__func__,__LINE__);
	mutex_lock(&adux1050->mutex);

	adux1050->read(adux1050->dev, INT_STATUS_REG,
		       &adux1050->int_status, DEF_WR);

	adux1050->low_status = GET_LOW_STATUS(adux1050->int_status);
	adux1050->high_status = GET_HIGH_STATUS(adux1050->int_status);
	adux1050->conv_status = GET_CONV_STATUS(adux1050->int_status);

	pr_info("HS:%x LS:%x CCS:%x\n", adux1050->high_status,
		adux1050->low_status, adux1050->conv_status);

	/* Handling Low threshold interrupt */
        if (adux1050->low_thresh_enable) {
		/* [PLATFORM]-Add Begin by TCTSZ.lizhi.wu, 2015.1.9*/
		if (adux1050->low_status)
		{
			//printk(KERN_CRIT"adux1050->low_status = %d", adux1050->low_status);
			pr_info("adux1050 low int detected!\n");
			adux1050->read(adux1050->dev, RESULT_STG0_REG, &data, DEF_WR);
			if ((data > 5000) && (data < 65535))
			{
				schedule_delayed_work(&adux1050->data_monitor_work, 1 * HZ);
			}
		}
		/* [PLATFORM]-Add End by TCTSZ.lizhi.wu, 2015.1.9*/
		/* [PLATFORM]-Removed Begin by TCTSZ.lizhi.wu, 2015.1.9*/
		/*
                low_status_change = ((adux1050->low_status) ^ (adux1050->prev_low_status));
                adux1050->prev_low_status = adux1050->low_status;
                if (low_status_change)
			low_threshold_int_check(adux1050, low_status_change);
		*/
		/* [PLATFORM]-Removed End by TCTSZ.lizhi.wu, 2015.1.9*/
        }

	/* Handling High threshold interrupt */
	if (adux1050->high_thresh_enable) {

		high_status_change = ((adux1050->high_status) ^
				      (adux1050->prev_high_status));
		adux1050->prev_high_status = adux1050->high_status;
		if (high_status_change)
		{
			//printk(KERN_CRIT"adux1050->high_status = %d", adux1050->high_status);
			pr_info("adux1050 high int detected!\n");
			high_threshold_int_check(adux1050, high_status_change);
		}
	}


	/* Handling Conversion complete interrupt */
	if (adux1050->conv_enable && adux1050->conv_status)
	{
		pr_info("adux1050 conv int detected!\n");
		conv_complete_cdc_fetch(adux1050);
	}

	mutex_unlock(&adux1050->mutex);
}

static int adux_gpio_config(struct adux1050_chip *adux1050)
{
	int ret = 0;

	struct device_node *node = adux1050->dev->of_node;

	printk("<2>""======%s,%d\n",__func__,__LINE__);

	psensor_data->enable_gpio = of_get_named_gpio(node, "enable-gpios", 0);
	if (psensor_data->enable_gpio < 0)
	{
		dev_err(adux1050->dev,
			"Looking up %s property in node %s failed. ret =  %d\n",
			"interrupt-gpios", node->full_name, psensor_data->enable_gpio);
		ret = -1;
	}
	else
	{
		ret = gpio_request(psensor_data->enable_gpio, "PSENSOR_ENABLE");
		if (ret)
		{
			dev_err(adux1050->dev,
				"%s: Failed to request gpio %d,ret = %d\n",
				__func__, psensor_data->enable_gpio, ret);

			ret = -1;
		}
		pr_info("The enable_gpio is %d\n", psensor_data->enable_gpio);
		/*
		ret = gpio_direction_output(psensor_data->enable_gpio,1);
		if (ret)
		{
			dev_err(adux1050->dev, "Unable to set direction for irq gpio [%d]\n",
				psensor_data->enable_gpio);
			gpio_free(psensor_data->enable_gpio);
			ret = -1;
		}*/

	}
	psensor_data->irq_gpio= of_get_named_gpio(node, "interrupt-gpios", 0);
	if (psensor_data->irq_gpio < 0)
	{
		dev_err(adux1050->dev,
			"Looking up %s property in node %s failed. ret =  %d\n",
			"interrupt-gpios", node->full_name, psensor_data->irq_gpio);
		ret = -1;
	}
	else
	{
		ret = gpio_request(psensor_data->irq_gpio, "PSENSOR_INTERRUPT");
		if (ret)
		{
			dev_err(adux1050->dev,
				"%s: Failed to request gpio %d,ret = %d\n",
				__func__, psensor_data->irq_gpio, ret);

			ret = -1;
		}
		ret = gpio_direction_input(psensor_data->irq_gpio);
		if (ret)
		{
			dev_err(adux1050->dev, "Unable to set direction for irq gpio [%d]\n", psensor_data->irq_gpio);
			gpio_free(psensor_data->irq_gpio);
			ret = -1;
		}
	}

	return ret;
}

static int adux_power_init(struct adux1050_chip *adux1050)
{
	int ret;
	struct device *dev = adux1050->dev;

	pr_info("---enter %s\n",__func__);
	psensor_data->vio = regulator_get(dev, "vio");
	if (IS_ERR(psensor_data->vio)) {
		ret = PTR_ERR(psensor_data->vio);
		dev_err(dev,
			"Regulator get failed vio ret=%d\n", ret);
		//goto reg_vdd_set;
		return ret;
	}

	if (regulator_count_voltages(psensor_data->vio) > 0) {
		ret = regulator_set_voltage(psensor_data->vio, ADUX_VIO_MIN_UV,
					   ADUX_VIO_MAX_UV);
		if (ret) {
			dev_err(dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			regulator_put(psensor_data->vio);
			return ret;
		}
	}
	return ret;
/*
reg_vdd_set:
	if (regulator_count_voltages(psensor_data->vdd) > 0)
		regulator_set_voltage(psensor_data->vdd, 0, ADUX_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(psensor_data->vdd);
*/
}

static int adux_power_deinit(struct adux1050_chip *adux1050)
{
	pr_info("---enter %s\n",__func__);

	if (IS_ERR(psensor_data->vio))
	{
		if (regulator_count_voltages(psensor_data->vio) > 0)
			regulator_set_voltage(psensor_data->vio,
					0, ADUX_VIO_MAX_UV);

		regulator_put(psensor_data->vio);
	}
	return 0;
}

static int adux_power_on(struct adux1050_chip *adux1050)
{
	int ret;
	struct device *dev = adux1050->dev;
	ret = gpio_direction_output(psensor_data->enable_gpio, 1);
        if (ret)
        {
                dev_err(adux1050->dev, "Unable to set direction for irq gpio [%d]\n",
                        psensor_data->enable_gpio);
                return -1;
        }
	if (!IS_ERR(psensor_data->vio)) {
		ret = regulator_enable(psensor_data->vio);
		if (ret) {
			dev_err(dev,
				"Regulator vio enable failed ret=%d\n", ret);
			return ret;
		}
	}
	ret = pinctrl_select_state(iqs128_pctrl.pinctrl,
                        iqs128_pctrl.gpio_state_active);
        if (ret)
                pr_err("%s:%d cannot set pin to active state",
                        __func__, __LINE__);
	psensor_data->power_on = true;
	return 0;
}

static int adux_power_off(struct adux1050_chip *adux1050)
{
	int ret;
	struct device *dev = adux1050->dev;

	ret = pinctrl_select_state(iqs128_pctrl.pinctrl,
                                iqs128_pctrl.gpio_state_deactive);
	if (ret)
		pr_err("%s:%d cannot set pin to suspend state\n", __func__, __LINE__);

	ret = gpio_direction_output(psensor_data->enable_gpio,0);
	if (ret)
	{
		dev_err(adux1050->dev, "Unable to set direction for irq gpio [%d]\n",
			psensor_data->enable_gpio);
		return -1;
	}
	if (!IS_ERR(psensor_data->vio)) {
		ret = regulator_disable(psensor_data->vio);
		if (ret)
			dev_err(dev,
				"Regulator vio disable failed ret=%d\n", ret);
		return ret;
	}
	psensor_data->power_on = false;
	return 0;
}

static int show_power_en(struct seq_file *s, void *p)
{
	struct device *dev = adux1050_enable_proc->data;
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	if (adux1050->dev_enable == ADUX1050_ENABLE)
        {
		seq_printf(s, "on\n");
        }
        else if (adux1050->dev_enable == ADUX1050_DISABLE)
        {
		seq_printf(s, "off\n");
        }
	else if (adux1050->dev_enable == ADUX1050_SUSPEND)
	{
		seq_printf(s, "suspend\n");
	}
	return 0;
}

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.21*/
static int adux1050_power_en_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_power_en, NULL);
}
static ssize_t adux1050_power_en_write(struct file *file,
				     const char __user *buffer,
				     size_t count, loff_t *pos)
{
	struct device *dev = adux1050_enable_proc->data;
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
        char *dev_state[2] = { "DISABLED", "ENABLED"};
        u16 val;
	u16 data = 0;

         if (!strncmp(buffer, "on", 2))
        {
                val = ADUX1050_ENABLE;
        }
        else if (!strncmp(buffer, "off", 3))
        {
                val = ADUX1050_DISABLE;
        }
        else
                return -1;
        if (adux1050->dev_enable == val) {
                dev_info(adux1050->dev, "%s - Device already in %s state\n",
                         __func__, dev_state[val]);
                return -1;
        }
	if ((val == ADUX1050_ENABLE) && (adux1050->dev_enable == ADUX1050_SUSPEND))
        {
                enable_irq(gpio_to_irq(psensor_data->irq_gpio));        /* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.24 */
                adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
                if (GET_PWR_MODE(data) != PWR_TIMED_CONV)
                {
                        data = SET_PWR_MODE(data, PWR_TIMED_CONV);
                        adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);
                }
                adux1050->dev_enable = ADUX1050_ENABLE;
                schedule_delayed_work(&adux1050->data_monitor_work, HZ / 2);
                dev_info(adux1050->dev, "ADUX1050 is enabled\n");
        }
        else if ((val == ADUX1050_DISABLE) && (adux1050->dev_enable == ADUX1050_ENABLE))
        {
                disable_irq(gpio_to_irq(psensor_data->irq_gpio));       /* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.24 */
                adux1050_standby(adux1050);
                adux1050->dev_enable = ADUX1050_SUSPEND;
		if (adux1050->high_thresh_enable)
                {
                        adux1050->prev_high_status = adux1050->high_status;
                        if (adux1050->send_event)
			{
                                send_psensor_uevent(1);
				#ifdef FUNCTION_WHILE_SLEEP
				wake_lock_timeout(&adux1050->adux1050_wakelock, 2 * HZ);
				#endif
			}
                }
                dev_info(adux1050->dev, "ADUX1050 is Disabled\n");
        }
        else
        {
                return -1;
        }
        return count;
}

static const struct file_operations adux1050_enable_fops = {
	.owner = THIS_MODULE,
	.open = adux1050_power_en_open,
	.read = seq_read,
	.write = adux1050_power_en_write,
};
/* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.1.21*/
/**
Device probe function  all initialization routines are handled here like
the ISR registration, Work creation,Input device registration, SYSFS
attributes creation etc.
@param i2c_client the i2c structure of the adux1050 device/client.
@param i2c_device_id The i2c_device_id for the supported i2c device.
@return 0 on success,and On failure -ENOMEM, -EINVAL ,etc., will be returned
*/
static int adux1050_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	s32 ret = -EINVAL;
#ifdef CONFIG_OF
//	s32 df_prop_length = 0;
//	const __be32 *irqf = NULL;
#endif
	//struct input_dev *input = NULL;	/* [PLATFORM]Removed by TCTSZ.lizhi.wu, 2014.12.30 */
	struct device *dev = NULL;
	struct adux1050_chip *adux1050 = NULL;

	ADUX1050_DRIVER_DBG("%s called", __func__);

	if ((client == NULL) || (&client->dev == NULL)) {
		pr_err("%s: Client/client->dev doesn't exist\n", __func__);
		return ret;
	} else {
		dev = &client->dev;
	}

	if (client->irq <= 0) {
		pr_err("%s: IRQ not configured!\n", __func__);
		return ret;
	}

	adux1050 = kzalloc(sizeof(*adux1050), GFP_KERNEL);
	if (!adux1050) {
		pr_err("%s: Memory alloc fail - Chip struct\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	psensor_data = kzalloc(sizeof(struct psensor_data), GFP_KERNEL);
	if (!psensor_data) {
		pr_err("Failed to allocate memory for psensor_data.\n");
		ret = -ENOMEM;
		goto err_kzalloc_mem;
	}

	adux1050->stg_info =
		kzalloc((sizeof(struct adux1050_stage_info) * TOTAL_STG),
			GFP_KERNEL);
	if (!adux1050->stg_info) {
		pr_err("%s: Memory Alloc fail - Stage info\n", __func__);
		return -ENOMEM;
	}

	if (dev->platform_data == NULL) {
		adux1050->pdata = &local_plat_data;
		pr_err("%s: Platform Data Not Found\n", __func__);

	adux1050->dev = dev;

#ifdef CONFIG_OF
		if (client->dev.of_node != NULL) {
/*			adux1050->dt_device_node = client->dev.of_node;
			pr_err("%s: DT node Found\n", __func__);
			irqf = of_get_property(adux1050->dt_device_node,
				  "adi,adux1050_irq_flags", &df_prop_length);
			if (irqf && (df_prop_length == sizeof(int))) {
				adux1050->pdata->irq_flags =
						be32_to_cpu(*irqf);
				pr_info("%s Usg irq_flag from DT %d\n",
					__func__, adux1050->pdata->irq_flags);

		}
*/
		adux_gpio_config(adux1050);
		} else {
			pr_info("%s - Usg local pltfm data\n", __func__);
		}
#endif

	} else {
		pr_info("%s - Pltfm Data Found\n", __func__);
		adux1050->pdata = dev->platform_data;
	}


	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.12.30 */
	psensor_data->psensor_dev = devm_kzalloc(adux1050->dev, sizeof(struct switch_dev), GFP_KERNEL);
        if (psensor_data->psensor_dev == NULL)
        {
		pr_err("%s:%d Unable to allocate memory\n",
                       __func__, __LINE__);
                goto err_kzalloc_mem;
        }
	psensor_data->psensor_dev->name = "psensor";
	psensor_data->psensor_dev->state = 1;		//far

	ret = switch_dev_register(psensor_data->psensor_dev);
	if (ret < 0){
		pr_err("register switch dev fail,ret=%d\n",ret);
		ret = -ENOMEM;
		goto err_kzalloc_mem1;
	}
	/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2014.12.30*/

	adux1050->read = adux1050_i2c_read;
	adux1050->write = adux1050_i2c_write;
	mutex_init(&adux1050->mutex);

	psensor_data->power_on = false;

	INIT_WORK(&psensor_data->usb_cali_work, usb_calibration_work_func);		//when usb pull in or out, this work would be scheduled
	ret = adux_power_init(adux1050);
	if (ret) {
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		ret = -EINVAL;
		goto err_kzalloc_mem1;
	}

	ret = iqs_pinctrl_init(dev);
        if (ret)
        {

                pr_err("%s:%d failed to init iqs_pinctrl handler\n",

                        __func__, __LINE__);

                return -1;

        }

	ret = adux_power_on(adux1050);
	if (ret) {
		dev_err(&client->dev, "Failed to enable sensor power\n");
		ret = -EINVAL;
		goto deinit_power_exit;
	}

	/* check if the device is existing by reading device id of ADUX1050 */
	/*Mod Begin by TCTSZ-WH,2015-1-7. Compatible iqs128 and adi.*/
	ret = adux1050_hw_detect(adux1050);
	if (ret)
	{

		ret = iqs128_init(dev);
		if (ret)		/*iqs128_init() failed*/
		{
			pr_err("No psensor detected.\n");
			goto err_free_irq;		/*deinit*/
		}
		return ret;
	}
//	adux_power_off(adux1050);
//	adux1050->dev_enable = ADUX1050_DISABLE;
	adux1050_i2c_driver.driver.pm= &adux1050_dev_pm_ops;
/*
	ret = iqs_pinctrl_init(dev);
	if (ret)
	{

		pr_err("%s:%d failed to init iqs_pinctrl handler\n",

			__func__, __LINE__);

		return -1;

	}
	ret = pinctrl_select_state(iqs128_pctrl.pinctrl,
                        iqs128_pctrl.gpio_state_active);
        if (ret)
        {
                pr_err("%s:%d cannot set pin to suspend state\n",
                        __func__, __LINE__);
                return -1;
        }
*/
	/*Mod End by TCTSZ-WH,2015-1-7. Compatible iqs128 and adi.*/

	i2c_set_clientdata(client, adux1050);
	INIT_WORK(&adux1050->work, adux1050_isr_work_fn);
	INIT_WORK(&adux1050->calib_work, adux1050_calibration);
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.6*/
	INIT_DELAYED_WORK(&adux1050->data_monitor_work, adux1050_data_monitor_work_fn);
	#ifdef USE_TRANCEBLITY
	INIT_DELAYED_WORK(&adux1050->trace_work, trace_work_func);
	#endif
	#ifdef ENVIRONMENT_CALIBRATION

	INIT_DELAYED_WORK(&adux1050->env_calib_work, adux1050_env_calib_work_fn);
	#endif
	/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.1.6*/
	/* [PLATFORM]-Removed-BEGIN by TCTSZ.lizhi.wu, 2014.12.30*/
	/*
	input = input_allocate_device();
	if (!input) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_kzalloc_mem;
	}

	adux1050->input = input;
	input_set_drvdata(adux1050->input, adux1050);
	input->name = DEVICE_NAME;
	set_bit(EV_MSC, input->evbit);
	input_set_capability(input, EV_MSC, MSC_RAW);

	ret = input_register_device(input);
	if (ret) {
		pr_err("%s: could not input_register_device(input);\n",
		       __func__);
		goto err_input_register_device;
	}*/
	/* [PLATFORM]-Removed-END by TCTSZ.lizhi.wu, 2014.12.30 */
	if (!adux1050->pdata->irq_flags)
		adux1050->pdata->irq_flags = IRQF_TRIGGER_FALLING;

	ADUX1050_DRIVER_DBG("%s - Before request_threaded_irq %d\n",
			    __func__, psensor_data->irq_gpio);
	ret = request_threaded_irq(gpio_to_irq(psensor_data->irq_gpio), NULL, adux1050_isr_thread,
				   IRQF_ONESHOT | adux1050->pdata->irq_flags,
				   DEVICE_NAME, adux1050);

	if (ret) {
		pr_err("%s: irq %d Driver init Failed", __func__,
		       psensor_data->irq_gpio);
		goto err_free_irq;
	}

	disable_irq(gpio_to_irq(psensor_data->irq_gpio));
	if (adux1050->pdata->irq_flags & IRQF_TRIGGER_RISING)
		adux1050->int_pol = ACTIVE_HIGH;
	else
		adux1050->int_pol = ACTIVE_LOW;

	ret = sysfs_create_group(&dev->kobj, &adux1050_attr_group);
	if (ret) {
		pr_err("%s: cound not register sensor sysfs\n", __func__);
		goto err_sysfs_create_input;
	}

	proc_dir = proc_mkdir("psensor", NULL);
	adux1050_enable_proc = proc_create_data("power_en", 664, proc_dir, &adux1050_enable_fops, adux1050->dev);
	if (adux1050_enable_proc == NULL)
	{
		pr_err("%s: Create adux1050_enable proc failed\n", __func__);
	}

	/* initialize and request sw/hw resources */
	adux1050_store_register_values(adux1050);
	adux1050->send_event = ADUX1050_ENABLE;

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.12.24 */
	#ifdef FUNCTION_WHILE_SLEEP
	device_init_wakeup(adux1050->dev, 1);
	wake_lock_init(&adux1050->adux1050_wakelock, WAKE_LOCK_SUSPEND, "adux1050_wake_lock");
	#endif
	adux1050_enable(adux1050);
	#ifndef USE_TRANCEBLITY
	adux1050_standby(adux1050);
	adux1050->dev_enable = ADUX1050_SUSPEND;
	#else
	schedule_delayed_work(&adux1050->trace_work, 5 * HZ);
	adux1050->dev_enable = ADUX1050_DISABLE;
	#endif
 
	p_sendevent = &adux1050->send_event;
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.12.24 */ 
 
#ifdef CONFIG_EVAL
	adux1050->power_mode_flag = ADUX1050_DISABLE;
#endif
	dev_info(adux1050->dev, "%s Completed\n", __func__);

	return 0;
err_sysfs_create_input:
	sysfs_remove_group(&dev->kobj, &adux1050_attr_group);
err_free_irq:
	free_irq(gpio_to_irq(psensor_data->irq_gpio), adux1050);
	//input_unregister_device(input);
	switch_dev_unregister(psensor_data->psensor_dev);		/* [PLATFORM]-Modified by TCTSZ.lizhi.wu, 2014.12.30 */
//err_input_register_device:
//	input_free_device(input);
deinit_power_exit:
	adux_power_deinit(adux1050);
err_kzalloc_mem1:
	devm_kfree(adux1050->dev, psensor_data->psensor_dev);
err_kzalloc_mem:
	kfree(adux1050);
	return ret;
}

/**
Removes the device.
This is used to remove the device or the I2C client from the system
@param client The client structure to be removed
@return 0 on success
*/
static int adux1050_i2c_remove(struct i2c_client *client)
{
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);
	u16 data = DISABLE_DEV_INT;
	pr_info("%s, Start\n", __func__);
	if (adux1050 != NULL) {
		if (adux1050->dev_enable == ADUX1050_ENABLE) {
			disable_irq(gpio_to_irq(psensor_data->irq_gpio));
			#ifdef FUNCiTION_WHILE_SLEEP
			if (device_may_wakeup(adux1050->dev))
			{
				disable_irq_wake(gpio_to_irq(psensor_data->irq_gpio));
			}
			#endif
			adux1050->write(adux1050->dev, INT_CTRL_REG,
					&data, DEF_WR);
			data = data | RESET_MASK;
			/* Reset should be reviewed for nesesity */
			adux1050->write(adux1050->dev, CTRL_REG,
					&data, DEF_WR);
		}
		sysfs_remove_group(&adux1050->dev->kobj,
				   &adux1050_attr_group);
		free_irq(gpio_to_irq(psensor_data->irq_gpio), adux1050);
		cancel_work_sync(&adux1050->work);
		cancel_work_sync(&adux1050->calib_work);
		cancel_delayed_work_sync(&adux1050->data_monitor_work);
		#ifdef ENVIRONMENT_CALIBRATION
		cancel_delayed_work_sync(&adux1050->env_calib_work);
		#endif
		//input_unregister_device(adux1050->input);
		switch_dev_unregister(psensor_data->psensor_dev);		/* [PLATFORM]-Modified by TCTSZ.lizhi.wu, 2014.12.30 */
		devm_kfree(adux1050->dev, psensor_data->psensor_dev);
		//input_free_device(adux1050->input);
		adux_power_off(adux1050);
		adux_power_deinit(adux1050);
		kfree(adux1050->stg_info); /* BJ */
		kfree(adux1050);
		i2c_set_clientdata(client, NULL);
	}
	pr_info("%s, END\n", __func__);
	return 0;
}

/**
Used similar to driver remove function but used as a shutdown call back
This is used to remove the device or the I2C client from the system
@param client The client ID to be removed
@return void
*/
void adux1050_shutdown(struct i2c_client *client)
{
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);
	u16 data = DISABLE_DEV_INT;
	pr_info("%s, Start\n", __func__);
	if (adux1050 != NULL) {
		if (adux1050->dev_enable == ADUX1050_ENABLE) {
			disable_irq(gpio_to_irq(psensor_data->irq_gpio));
			#ifdef FUNCiTION_WHILE_SLEEP
			if (device_may_wakeup(adux1050->dev))
			{
				disable_irq_wake(gpio_to_irq(psensor_data->irq_gpio));
			}
			#endif
			adux1050->write(adux1050->dev, INT_CTRL_REG,
					&data, DEF_WR);
		}
		sysfs_remove_group(&adux1050->dev->kobj,
				   &adux1050_attr_group);
		free_irq(gpio_to_irq(psensor_data->irq_gpio), adux1050);
		cancel_work_sync(&adux1050->work);
		cancel_work_sync(&adux1050->calib_work);
		cancel_delayed_work_sync(&adux1050->data_monitor_work);
                #ifdef ENVIRONMENT_CALIBRATION
                cancel_delayed_work_sync(&adux1050->env_calib_work);
                #endif
		//input_unregister_device(adux1050->input);
		//input_free_device(adux1050->input);		/* [PLATFORM]-Modified by TCTSZ.lizhi.wu, 2014.12.30 */
		switch_dev_unregister(psensor_data->psensor_dev);
		devm_kfree(adux1050->dev, psensor_data->psensor_dev);
		kfree(adux1050->stg_info);
		kfree(adux1050);
		i2c_set_clientdata(client, NULL);
	}
	pr_info("%s, END\n", __func__);
}

/**
Device suspend PM operation call back function
@dev Device structure for handling the power management.
@return Zero on success
*/
static int adux1050_i2c_suspend(struct device *dev)
{
	int err = 0;

	struct i2c_client *client = to_i2c_client(dev);
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);

	dev_info(adux1050->dev, "%s,check (%d)\n",
		 __func__, adux1050->dev_enable);
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.15*/
	#ifdef FUNCTION_WHILE_SLEEP
	if (device_may_wakeup(dev))
	{
		enable_irq_wake(gpio_to_irq(psensor_data->irq_gpio));
	}
	#else
	/* To put device in STANDBY Mode */
	if (adux1050->dev_enable == ADUX1050_ENABLE) {
	//	adux1050_disable(adux1050);
		disable_irq(gpio_to_irq(psensor_data->irq_gpio));
		adux1050_standby(adux1050);
		adux1050->dev_enable = ADUX1050_SUSPEND;
	}
	#endif
	/* [PLATFORM]-Removed-end by TCTSZ.lizhi.wu, 2015.1.15*/
	return err;
}

/**
Device resume PM operation call back function
@dev Device structure for handling the power management.
@return Zero on success
*/
static int adux1050_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);
	//u16 data = 0;

	dev_info(adux1050->dev, "%s,check (%d)\n",
		 __func__, adux1050->dev_enable);

	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.1.15*/
	#ifdef FUNCTION_WHILE_SLEEP
	if (device_may_wakeup(dev))
	{
		disable_irq_wake(gpio_to_irq(psensor_data->irq_gpio));
	}
	#else
	if (adux1050->dev_enable == ADUX1050_SUSPEND)
	{
		enable_irq(gpio_to_irq(psensor_data->irq_gpio));        /* [PLATFORM]Modified by TCTSZ.lizhi.wu, 2014.12.24 */
		adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
                if (GET_PWR_MODE(data) != PWR_TIMED_CONV)
                {
                        data = SET_PWR_MODE(data, PWR_TIMED_CONV);
                        adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);
                }
//		adux1050_enable(adux1050);
		schedule_delayed_work(&adux1050->data_monitor_work, HZ / 2);
		adux1050->dev_enable = ADUX1050_ENABLE;
	}
	#endif
	/* [PLATFORM]-Removed-END by TCTSZ.lizhi.wu, 2015.1.15*/
	return 0;
}

/**
Device ID table for the ADUX1050 driver
*/
static const struct of_device_id adux1050_of_id[] = {
	{.compatible = "adi,adux1050"},
	{}
};

/**
Device ID table for the ADUX1050 driver
*/
static const struct i2c_device_id adux1050_id[] = {
	{ "adux1050", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, adux1050_id);

/**
  The file Operation table for power
*/
static const struct dev_pm_ops adux1050_dev_pm_ops = {
	.suspend = adux1050_i2c_suspend,
	.resume = adux1050_i2c_resume,
};

/**
I2C driver structure containing the function callbacks,
driver name and ID tables
*/
struct i2c_driver adux1050_i2c_driver = {
	.driver = {
		.name = PSENSOR_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifndef COMPATIBLE_NEEDED		/*MOD by TCTSZ-WH,2015-1-9. Define .pm only if adux1050 detected.*/
		.pm = &adux1050_dev_pm_ops,
#endif
#ifdef	CONFIG_OF
		.of_match_table = adux1050_of_id,
#endif
	},
	.probe    = adux1050_probe,
	.shutdown = adux1050_shutdown,
	.remove   = adux1050_i2c_remove,
	.id_table = adux1050_id,
};

/**
This is an init function called during module insertion.
calls in turn i2c driver probe function
*/
static __init int adux1050_module_init(void)
{
	pr_info("%s,START\n", __func__);
	return i2c_add_driver(&adux1050_i2c_driver);
}
module_init(adux1050_module_init);

/**
This is an exit function called during module removal --
calls in turn i2c driver delete function
*/
static __exit void adux1050_module_exit(void)
{
	i2c_del_driver(&adux1050_i2c_driver);
}

module_exit(adux1050_module_exit);
MODULE_DESCRIPTION("Analog Devices ADUX1050 Driver");
MODULE_AUTHOR("Analog Devices");
MODULE_LICENSE("GPL");

