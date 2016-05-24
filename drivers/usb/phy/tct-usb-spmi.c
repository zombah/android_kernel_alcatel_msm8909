
#define pr_fmt(fmt) ">>TCT USB<< %s,%d. " fmt,__func__,__LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/ctype.h>

#include "tct-usb-spmi.h"

static bool internal_charger = true;
static u8   chg_option = 0;

struct tct_usb_spmi_device
{
	struct device			*dev;
	struct spmi_device		*spmi;
	struct dentry 			*debugfs;

	u16						resoure_base;
	u16						resoure_end;
};
struct tct_usb_spmi_device *this_device = NULL;

//struct spmi_device *tct_usb_spmi_device = NULL;


#define TCT_USB_SPMI_NAME		"tct,usb-get-spmi"

#define TCT_READ_BUFFER_SIZE 	64

#define TCT_USB_SPMI_CHG_OPTION_REG 0x1008

/*
0: EXT_CHARGER
1: PMIC_CHARGER
*/
bool tct_usb_get_spmi_chg_option(void)
{
	return internal_charger;
}

static int tct_usb_read_spmi(struct tct_usb_spmi_device *device, u16 base,
				u8 *val, int count)
{
	struct spmi_device *spmi = device->spmi;
	int rc = 0;

	if((base < device->resoure_base) || (base > device->resoure_end))
	{
		pr_err("Read SPMI out of resoure.0x%x-<0x%x-0x%x>\n",
			base, device->resoure_base, device->resoure_end);
		//return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static ssize_t tct_chg_option_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tct_usb_spmi_device *device = dev_get_drvdata(dev);
	u8 val, ret;

	ret = tct_usb_read_spmi(device, TCT_USB_SPMI_CHG_OPTION_REG, &val, 1);
	if (ret)
		pr_err("SPMI read failed rc=%d\n", ret);

	return snprintf(buf, TCT_READ_BUFFER_SIZE, "chg_option boot:0x%x, now:0x%x!\n", 
		chg_option, val);
}

static ssize_t tct_chg_option_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	pr_debug(">>%s<< Unsupport write function now!\n", __func__);

	return size;
}

static DEVICE_ATTR(tct_chg_option, 0664, tct_chg_option_show, tct_chg_option_store);

static int tct_usb_parse_resources(struct tct_usb_spmi_device *device,
				struct spmi_device *spmi)
{
	//struct spmi_resource *spmi_resource;
	struct resource *resource;
	//int rc;

	resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
	if (!(resource && resource->start)) 
	{
		pr_err("node IO resource absent!\n");
		return -ENXIO;
	}

	device->resoure_base = resource->start;
	device->resoure_end	 = resource->end;

	device->dev = &spmi->dev;
	device->spmi = spmi;

	printk(KERN_WARNING "----%s<%d>---- resource base:0x%x, offset:0x%x!\n",
		__func__, __LINE__, resource->start, resource->end);

	return 0;
}

static int tct_usb_probe(struct spmi_device *spmi)
{
	struct tct_usb_spmi_device *device;
	int rc = 0;

	printk(KERN_WARNING "----%s<%d>---- START\n", __func__, __LINE__);

	device = devm_kzalloc(&spmi->dev, sizeof(struct tct_usb_spmi_device), GFP_KERNEL);
	if (!device) {
		pr_err("kzalloc() failed.\n");
		return -ENOMEM;
	}

	rc = tct_usb_parse_resources(device, spmi);
	if (rc) {
		pr_err("Error registering spmi resource rc=%d\n", rc);
		return rc;
	}	

	dev_set_drvdata(&spmi->dev, device);

	rc = tct_usb_read_spmi(device, TCT_USB_SPMI_CHG_OPTION_REG, &chg_option, 1);
	if (rc)
		pr_err("SPMI read failed rc=%d\n", rc);

	internal_charger = 0x80 == (chg_option & 0x80);

	printk(KERN_WARNING "----%s<%d>---- END. chg_option:0x%x, internal_charger:%s\n",
		 __func__, __LINE__, chg_option, internal_charger?"true":"false");

#if 0
	rc = tct_usb_read_spmi(device, TCT_USB_SPMI_CHG_OPTION_REG + 1, &, 1);
	if (rc)
		pr_err("SPMI read failed rc=%d\n", rc);
#endif		

	this_device = device;

	rc = device_create_file(&spmi->dev, &dev_attr_tct_chg_option);
	if (rc) {
		dev_err(&spmi->dev, "sys file creation failed\n");
		return rc;
	}

	return rc;		
}

static int tct_usb_remove(struct spmi_device *spmi)
{
	device_remove_file(&spmi->dev, &dev_attr_tct_chg_option);

	this_device->spmi = NULL;
	dev_set_drvdata(&spmi->dev, NULL);
	devm_kfree(&spmi->dev, this_device);
	this_device = NULL;

	return 0;
}

static struct of_device_id tct_usb_match_table[] = {
	{ .compatible = TCT_USB_SPMI_NAME },
	{}
};

static struct spmi_driver tct_usb_driver = {
	.probe		= tct_usb_probe,
	.remove		= tct_usb_remove,
	.driver		= {
		.name		= TCT_USB_SPMI_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= tct_usb_match_table,
	},
};

static int __init tct_usb_init(void)
{
	return spmi_driver_register(&tct_usb_driver);
}
module_init(tct_usb_init);

static void __exit tct_usb_exit(void)
{
	return spmi_driver_unregister(&tct_usb_driver);
}
module_exit(tct_usb_exit);

MODULE_DESCRIPTION("TCT USB get spmi information");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" TCT_USB_SPMI_NAME);