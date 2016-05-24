
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
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
#include <linux/regulator/consumer.h>
#include "psensor.h"
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>

//static struct platform_device *psensor_pdev;
extern struct psensor_data *psensor_data;
void send_psensor_uevent(enum PSENSOR_STATUS val);
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.12.18 add psensor power control*/
#define PSENSOR_POWER_CONTROL

#ifdef PSENSOR_POWER_CONTROL
u8 psensor_power_on  = 0;
static struct proc_dir_entry *proc_psensor_dir = NULL;
static struct proc_dir_entry *proc_name = NULL;
/*
#define IQS_PINCTRL_STATE_DEACTIVE "iqs128_int_deactive"
#define IQS_PINCTRL_STATE_ACTIVE "iqs128_int_active"

struct iqs_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_deactive;
};*/

struct iqs_pinctrl_info iqs128_pctrl;
int iqs_pinctrl_init(struct device *dev)
{
	iqs128_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(iqs128_pctrl.pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	iqs128_pctrl.gpio_state_active = pinctrl_lookup_state(
					       iqs128_pctrl.pinctrl,
					       IQS_PINCTRL_STATE_ACTIVE);

	if (IS_ERR_OR_NULL(iqs128_pctrl.gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	iqs128_pctrl.gpio_state_deactive =  pinctrl_lookup_state(
						iqs128_pctrl.pinctrl,
						IQS_PINCTRL_STATE_DEACTIVE);

	if (IS_ERR_OR_NULL(iqs128_pctrl.gpio_state_deactive)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static ssize_t psensor_power_en_store(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	if (!strncmp(buf, "on", 2))
	{
		psensor_power_on = 1;
		schedule_work(&psensor_data->power_control_work);
	}
	else if (!strncmp(buf, "off", 3))
	{
		psensor_power_on = 0;
		schedule_work(&psensor_data->power_control_work);
	}
	return size;
}

static int psensor_power_en_show(struct seq_file *m, void *v)
{
        if (psensor_power_on == 1)
        {
                seq_printf(m, "on\n");
        }
        else if (psensor_power_on == 0)
        {
                seq_printf(m, "off\n");
        }
        return 0;
}

static int psensor_power_en_open(struct inode *inode, struct file *file)
{
	return single_open(file, psensor_power_en_show, NULL);
}

//static DEVICE_ATTR(power_en, S_IRUGO | S_IWUSR | S_IWGRP, psensor_power_en_show, psensor_power_en_store);
static const struct file_operations psensor_power_en_fops = {
	.open = psensor_power_en_open,
	.write = psensor_power_en_store,
	.read = seq_read,
	.owner = THIS_MODULE,
};

static void power_control_work_func(struct work_struct *work)
{
	int ret;

	if (psensor_power_on == 1)
	{
		ret = gpio_direction_output(psensor_data->enable_gpio,1);
		if (ret)
		{
			printk(KERN_CRIT "Unable to set direction for enable gpio [%d]\n",
				psensor_data->enable_gpio);
			psensor_power_on = 0;
			return;
		}

	ret = pinctrl_select_state(iqs128_pctrl.pinctrl,
			iqs128_pctrl.gpio_state_active);
	if (ret)
		pr_err("%s:%d cannot set pin to active state",
			__func__, __LINE__);

		printk(KERN_CRIT"Psensor power on\n");
	}
	else if (psensor_power_on == 0)
	{
		ret = gpio_direction_output(psensor_data->enable_gpio, 0);
		if (ret)
		{
			printk( KERN_CRIT"Unable to set direction for irq gpio [%d]\n",
				psensor_data->enable_gpio);
			psensor_power_on = 1;
			return;
		}

		ret = pinctrl_select_state(iqs128_pctrl.pinctrl,
				iqs128_pctrl.gpio_state_deactive);
		if (ret)
			pr_err("%s:%d cannot set pin to suspend state\n",
				__func__, __LINE__);

		printk(KERN_CRIT"Psensor power off\n");
	}
}
#endif

#if 0
static int psensor_gpio_config(struct device *dev)
{
	int ret = 0;

	struct device_node *node = dev->of_node;

	psensor_data->enable_gpio = of_get_named_gpio(node, "enable-gpios", 0);
	if (psensor_data->enable_gpio < 0)
	{
		dev_err(dev,
			"Looking up %s property in node %s failed. ret =  %d\n",
			"interrupt-gpios", node->full_name, psensor_data->enable_gpio);
		ret = -1;
	}
	else
		{
		ret = gpio_request(psensor_data->enable_gpio, "PSENSOR_ENABLE");
		if (ret)
		{
			dev_err(dev,
				"%s: Failed to request gpio %d,ret = %d\n",
				__func__, psensor_data->enable_gpio, ret);
		
			ret = -1;
		}
		ret = gpio_direction_output(psensor_data->enable_gpio,1);
		if (ret)
		{
			dev_err(dev, "Unable to set direction for irq gpio [%d]\n",
				psensor_data->enable_gpio);
			gpio_free(psensor_data->enable_gpio);
			ret = -1;
		}
	
		}
	
	psensor_data->irq_gpio= of_get_named_gpio(node, "interrupt-gpios", 0);
	if (psensor_data->irq_gpio < 0)
	{
		dev_err(dev,
			"Looking up %s property in node %s failed. ret =  %d\n",
			"interrupt-gpios", node->full_name, psensor_data->irq_gpio);
		ret = -1;
	}
	else
	{
		ret = gpio_request(psensor_data->irq_gpio, "PSENSOR_INTERRUPT");
		if (ret)
		{
			dev_err(dev,
				"%s: Failed to request gpio %d,ret = %d\n",
				__func__, psensor_data->irq_gpio, ret);
		
			ret = -1;
		}
		ret = gpio_direction_input(psensor_data->irq_gpio);
		if (ret)
		{
			dev_err(dev, "Unable to set direction for irq gpio [%d]\n", psensor_data->irq_gpio);
			gpio_free(psensor_data->irq_gpio);
			ret = -1;
		}
	}

	return ret;
}
#endif
static irqreturn_t iqs128_int_handler(int irq, void *dev)
{
	printk("<2>""---%s,gpio_get_value(irq_gpio) = %d\n",__func__,gpio_get_value(psensor_data->irq_gpio));
	psensor_data->p_sensor_value = gpio_get_value(psensor_data->irq_gpio);

	if (!work_pending(&psensor_data->psensor_work)) {
		queue_work(psensor_data->psensor_wq, &psensor_data->psensor_work);
	}

	return IRQ_HANDLED;
}

void send_psensor_uevent(enum PSENSOR_STATUS val)
{
	char *envp[2];
	char psensor[20];
	if(psensor_data == NULL)
		return;

	psensor_data->psensor_dev->state = val;
	snprintf(psensor, sizeof(psensor), "SWITCH_STATE=%d", psensor_data->psensor_dev->state);
	envp[0] = psensor;
	envp[1] = NULL;
	
	if(!psensor_data->psensor_dev->state)
		kobject_uevent_env(&psensor_data->psensor_dev->dev->kobj, KOBJ_ADD, envp);
	else
		kobject_uevent_env(&psensor_data->psensor_dev->dev->kobj, KOBJ_REMOVE, envp);
}

static void do_psensor_work(struct work_struct *work)
{
	send_psensor_uevent(psensor_data->p_sensor_value);
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.11.25 */
	wake_lock_timeout(&psensor_data->psensor_wake_lock, 2 * HZ);
        /* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2014.11.25 */
}

int iqs128_init(struct device *dev)
{
	int ret = 0;

	pr_info("enter %s\n",__func__);
	gpio_direction_output(psensor_data->enable_gpio,0);	/*Disable iqs128 when power on.*/
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
	INIT_WORK(&psensor_data->psensor_work, do_psensor_work);
	psensor_data->psensor_wq = create_singlethread_workqueue("psensor_wq");
	if (!psensor_data->psensor_wq) {
		pr_err("create thread error, line: %d\n", __LINE__);
		return -ENOMEM;
	}
	
	/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.11.25 */
#ifdef PSENSOR_POWER_CONTROL
	proc_psensor_dir = proc_mkdir("psensor", NULL);
	if (proc_psensor_dir == NULL)
	{
		pr_err( "create psensor file failed ");
		return -1;
	}
	proc_name = proc_create("power_en", S_IRUGO | S_IWUSR | S_IWGRP, proc_psensor_dir, &psensor_power_en_fops);
	if (proc_name == NULL)
	{
		pr_err("create power_en file failed ");
		remove_proc_entry("psensor", NULL);
		return -1;
	}
	INIT_WORK(&psensor_data->power_control_work, power_control_work_func);
#endif

	ret = request_irq(gpio_to_irq(psensor_data->irq_gpio), iqs128_int_handler, IRQ_TYPE_EDGE_BOTH , "iqs128_work", NULL);
	if(ret < 0)
	{
		pr_err("request_irq failed. irq = %d\n",gpio_to_irq(psensor_data->irq_gpio));
		goto exit_free_irq;
	}

	enable_irq_wake(gpio_to_irq(psensor_data->irq_gpio));
	wake_lock_init(&psensor_data->psensor_wake_lock, WAKE_LOCK_SUSPEND, "psensor_wake_lock");

	return 0;

exit_free_irq:
	free_irq(gpio_to_irq(psensor_data->irq_gpio),NULL);

	return ret;
}

#if 0
static int iqs128_probe(struct platform_device *pdev)
{
	int ret;

	printk("#####enter %s\n",__func__);

	psensor_data = kzalloc(sizeof(struct psensor_data), GFP_KERNEL);
	if (!psensor_data) {
		pr_err("Failed to allocate memory for psensor_data.\n");
		return -ENOMEM;
	}

	psensor_data->psensor_dev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	 if (psensor_data->psensor_dev == NULL)
	{
			pr_err("%s:%d Unable to allocate memory\n",
				__func__, __LINE__);
			return -ENOMEM;
	}

	printk("<2>""%s: start config gpio\n",__func__);
	psensor_gpio_config(&pdev->dev);
/*
	ret = iqs_pinctrl_init(&pdev->dev);
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
	psensor_data->p_sensor_value = 1;
	psensor_data->psensor_dev->name = "psensor";
	psensor_data->psensor_dev->state = PSENSOR_STATUS_FAR;

	ret = switch_dev_register(psensor_data->psensor_dev);
	if (ret < 0){
		pr_err("register switch dev fail,ret=%d\n",ret);
		return -2;
	}
	send_psensor_uevent(PSENSOR_STATUS_FAR);
	psensor_pdev = pdev;

	ret = iqs128_init();

	enable_irq_wake(gpio_to_irq(psensor_data->irq_gpio));
	wake_lock_init(&psensor_data->psensor_wake_lock, WAKE_LOCK_SUSPEND, "psensor_wake_lock");

	return ret;
}


static int iqs128_remove(struct platform_device *pdev)
{
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.11.25 */
	wake_lock_destroy(&psensor_data->psensor_wake_lock);
	remove_proc_entry("psensor", NULL);
	remove_proc_entry("psensor_en", proc_psensor_dir);
/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2014.11.25 */
	switch_dev_unregister(psensor_data->psensor_dev);
	destroy_workqueue(psensor_data->psensor_wq);
	gpio_free(psensor_data->irq_gpio);
	free_irq(psensor_data->irq_gpio,psensor_data);

	gpio_free(psensor_data->enable_gpio);

	devm_kfree(&pdev->dev, psensor_data->psensor_dev);
	devm_kfree(&pdev->dev, psensor_data);
	return 0;
}
#endif

