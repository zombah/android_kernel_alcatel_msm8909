#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

char ssr_buf[1024] = "subsystem restart reason:\n";

 static int ssr_record_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", ssr_buf);
	return 0;
}

static int ssr_record_open(struct inode *inode, struct file *file)
{
	return single_open(file, ssr_record_show, NULL);
}

static const struct file_operations ssr_record_proc_fops = {
	.open           = ssr_record_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init init_ssr_record_proc(void)
{

	if (!proc_create("ssr_record", S_IRWXU, NULL, &ssr_record_proc_fops))
	printk(KERN_ERR "init_mmc_log_proc create_proc_entry fails\n");
	return 0;
}

module_init(init_ssr_record_proc);
