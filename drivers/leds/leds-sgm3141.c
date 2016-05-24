
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>

#include <linux/delay.h>

/* #define CONFIG_GPIO_FLASH_DEBUG */
#undef CDBG
#ifdef CONFIG_GPIO_FLASH_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define LED_GPIO_FLASH_DRIVER_NAME	"sgm3141"
#define LED_TRIGGER_DEFAULT		"none"

/* NOTE: According to informations from SGM FAE, sgm3141 has HW safety timer of about 290ms inside;
however sgm3141B doesn't have the HW safety timer, but it has the thermal shutdown protection.
so the safety time defined here should be the minum value between the LED's max flash time  and 290ms.
*/
#define FLASH_SAFETY_TIME (200)

struct flash_driver_info {
	int flash_en;
	int flash_torch;
	int flash_on;

	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct pinctrl *pinctrl;

	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	struct mutex lock;
	struct delayed_work flash_safety_work;
};

static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

static void led_gpio_control(struct flash_driver_info *flash_led, int flash_en, int flash_torch)
{
	int rc = 0;

//	mutex_lock(&flash_led->lock);
/* disable the driver ic first */
	rc = gpio_direction_output(flash_led->flash_en, 0);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->flash_en, rc);
	}

	rc = gpio_direction_output(flash_led->flash_torch, flash_torch);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->flash_torch, rc);
	}

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->flash_en, rc);
	}
//	mutex_unlock(&flash_led->lock);

}

static void flash_safety_time_control(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct flash_driver_info *flash_led;

	delay_work = to_delayed_work(work);
	flash_led = container_of(delay_work, struct flash_driver_info, flash_safety_work);

	if ( (1 == flash_led->flash_on) && flash_led ) {
		led_gpio_control(flash_led, 0, 0);
		flash_led->flash_on = 0;
	} else
		printk("%s flash_led NULL \n", __func__);

}

static void led_gpio_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct flash_driver_info *flash_led =NULL;

	if (0 == strncmp(led_cdev->name, "led-flash", strlen("led-flash"))) {

		flash_led = container_of(led_cdev, struct flash_driver_info, cdev_flash);
		if (flash_led) {
			if (value > 0) {
				led_gpio_control(flash_led, 1, 1);
				flash_led->flash_on = 1;
				schedule_delayed_work(&flash_led->flash_safety_work,
					msecs_to_jiffies(FLASH_SAFETY_TIME));
			} else {
				led_gpio_control(flash_led, 0, 0);
				if (1 == flash_led->flash_on) {
					flash_led->flash_on = 0;
					cancel_delayed_work_sync(&flash_led->flash_safety_work);
				}
			}
		} else
			printk("%s line(%d) NULL flash_led \n", __func__, __LINE__);

	} else if (0 == strncmp(led_cdev->name, "led-torch", strlen("led-torch"))) {

		flash_led = container_of(led_cdev, struct flash_driver_info, cdev_torch);

		if (flash_led) {
			if (value > 0)
				led_gpio_control(flash_led, 1, 0);
			else
				led_gpio_control(flash_led, 0, 0);
		} else
			printk("%s line(%d) NULL torch_led \n", __func__, __LINE__);
	}

}

static enum led_brightness led_gpio_brightness_get(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int led_gpio_flash_config(struct platform_device *pdev, struct device_node *temp)
{
	int rc = 0;
	const char *temp_str;
	struct flash_driver_info *flash_led = NULL;
	struct led_classdev *cdev = NULL;

	flash_led = platform_get_drvdata(pdev);

	if (!flash_led) {
		printk(" %s NULL pointer at flash_led \n", __func__);
		return -1;
	}

	rc = of_property_read_string(temp, "linux,name", &temp_str);
	if (rc) {
		printk("%s: Failed to read linux name. rc = %d\n", __func__, rc);
		return -2;
	}

	if (0 == strncmp(temp_str, "led-torch", strlen("led-torch"))) {
		cdev = &flash_led->cdev_torch;
	} else if (0 == strncmp(temp_str, "led-flash", strlen("led-flash"))) {
		cdev = &flash_led->cdev_flash;
	}

	if (!cdev) {
		printk("%s NULL pointer at cdev \n", __func__);
		return -3;
	}

	cdev->name = temp_str;
	rc = of_property_read_string(temp, "linux,name", &cdev->name);
	if (rc) {
		printk("%s: Failed to read linux name. rc = %d\n", __func__, rc);
		return -4;
	}

	cdev->default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(temp, "linux,default-trigger", &temp_str);
	if (!rc)
		cdev->default_trigger = temp_str;

	cdev->max_brightness = LED_FULL;
	cdev->brightness_set = led_gpio_brightness_set;
	cdev->brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, cdev);
	if (rc) {
		printk("%s: Failed to register led dev. rc = %d\n", __func__, rc);
		led_classdev_unregister(cdev);
		return -5;
	}

	return 0;
}

static int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct flash_driver_info *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *temp = NULL;
	int num_leds = 0;

	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct flash_driver_info),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		printk("%s:%d Unable to allocate memory \n", __func__, __LINE__);
		return -ENOMEM;
	}

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		printk("%s:%d failed to get pinctrl\n", __func__, __LINE__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->pins_active = pinctrl_lookup_state(flash_led->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(flash_led->pins_active)) {
	    printk("%s:%d Failed to lookup pinctrl default state \n", __func__, __LINE__);
	    return PTR_ERR(flash_led->pins_active);
	}

	flash_led->pins_sleep = pinctrl_lookup_state(flash_led->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(flash_led->pins_sleep)) {
	    printk("%s:%d Failed to lookup pinctrl sleep state\n", __func__, __LINE__);
	    return PTR_ERR(flash_led->pins_sleep);
	}

	rc = pinctrl_select_state(flash_led->pinctrl, flash_led->pins_active);
	if (rc) {
	    printk("%s: Can not set %s pins rc=%d \n", __func__, PINCTRL_STATE_DEFAULT, rc);
	}

	flash_led->flash_en = of_get_named_gpio(node, "leds,flash-en", 0);

	if (flash_led->flash_en < 0) {
		printk("%s Looking up %s property in node %s failed. rc =  %d \n", __func__,
			"flash-en", node->full_name, flash_led->flash_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			printk("%s: Failed to request gpio %d,rc = %d\n", __func__, flash_led->flash_en, rc);
			goto error;
		}
	}

	flash_led->flash_torch = of_get_named_gpio(node, "leds,flash-torch", 0);
	if (flash_led->flash_torch < 0) {
		printk("%s Looking up %s property in node %s failed. rc =  %d \n", __func__,
			"torch-en", node->full_name, flash_led->flash_torch);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_torch, "TORCH_EN");
		if (rc) {
			printk("%s: Failed to request gpio %d, rc = %d\n",
				__func__, flash_led->flash_torch, rc);
			goto error;
		}
	}

	gpio_direction_output(flash_led->flash_torch, 0);
	gpio_direction_output(flash_led->flash_en, 0);
	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (2 != num_leds)
		return -3;

	platform_set_drvdata(pdev, flash_led);

	if (2 == num_leds)
		for_each_child_of_node(node, temp)
			led_gpio_flash_config(pdev, temp);

	INIT_DELAYED_WORK(&flash_led->flash_safety_work, flash_safety_time_control);
	mutex_init(&flash_led->lock);
	flash_led->flash_on = 0;
	printk("%s:probe successfully!\n", __func__);
	return 0;

error:
	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

static int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct flash_driver_info *flash_led =
	    (struct flash_driver_info *)platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&flash_led->flash_safety_work);
	mutex_destroy(&flash_led->lock);

	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_flash_probe,
	.remove = led_gpio_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

static int __init led_gpio_flash_init(void)
{
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	return platform_driver_unregister(&led_gpio_flash_driver);
}

module_init(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("QCOM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-sgm3141");
