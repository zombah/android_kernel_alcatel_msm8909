/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/qcom/memory_dump.h>

#ifdef CONFIG_JRD_BUTTON_RAMCONSOLE_WDT
//add by jch for watch dog ramdump PR-802266
#ifdef CONFIG_TCT_WATCHDOG_CTX_PRINT
#include <linux/pstore_ram.h>
#include <linux/kallsyms.h>
#include <linux/dma-mapping.h>
#include <soc/qcom/socinfo.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#endif
//end add by jch for watch dog ramdump PR-802266
#endif

#include <soc/qcom/scm.h>

#define MSM_DUMP_TABLE_VERSION		MSM_DUMP_MAKE_VERSION(2, 0)

#define SCM_CMD_DEBUG_LAR_UNLOCK	0x4

struct msm_dump_table {
	uint32_t version;
	uint32_t num_entries;
	struct msm_dump_entry entries[MAX_NUM_ENTRIES];
};

struct msm_memory_dump {
	uint64_t table_phys;
	struct msm_dump_table *table;
};

static struct msm_memory_dump memdump;

#ifdef CONFIG_JRD_BUTTON_RAMCONSOLE_WDT
//add by jch for watch dog ramdump PR-802266  
#ifdef CONFIG_TCT_WATCHDOG_CTX_PRINT
#define TZBSP_DUMP_CTX_MAGIC		0x42445953
#define TZBSP_DUMP_CTX_VERSION		0x11//0x3
#define MAX_CPU_CTX_PAGE_SIZE		1024
struct cpu32_ctxt_regs_type
{
    uint64_t u1;
    uint64_t u2;
    uint64_t r0;
    uint64_t r1;
    uint64_t r2;
    uint64_t r3;
    uint64_t r4;
    uint64_t r5;
    uint64_t r6;
    uint64_t r7;
    uint64_t r8;
    uint64_t r9;
    uint64_t r10;
    uint64_t r11;
    uint64_t r12;
    uint64_t r13_usr;
    uint64_t r14_usr;
    uint64_t r13_hyp;
    uint64_t r14_irq;
    uint64_t r13_irq;
    uint64_t r14_svc;
    uint64_t r13_svc;
    uint64_t r14_abt;
    uint64_t r13_abt;
    uint64_t r14_und;
    uint64_t r13_und;
    uint64_t r8_fiq;
    uint64_t r9_fiq;
    uint64_t r10_fiq;
    uint64_t r11_fiq;
    uint64_t r12_fiq;
    uint64_t r13_fiq;
    uint64_t r14_fiq;
    uint64_t pc;
    uint64_t cpsr;
    uint64_t r13_mon;
    uint64_t r14_mon;
    uint64_t r14_hyp;
    uint64_t _reserved;
    uint64_t __reserved1;
    uint64_t __reserved2;
    uint64_t __reserved3;
    uint64_t __reserved4;
};

struct cpu32_secure_ctxt_regs_type
{
    uint64_t r0;
    uint64_t r1;
    uint64_t r2;
    uint64_t r3;
    uint64_t r4;
    uint64_t r5;
    uint64_t r6;
    uint64_t r7;
    uint64_t r8;
    uint64_t r9;
    uint64_t r10;
    uint64_t r11;
    uint64_t r12;
    uint64_t r13_usr;
    uint64_t r14_usr;
    uint64_t r13_hyp;
    uint64_t r14_irq;
    uint64_t r13_irq;
    uint64_t r14_svc;
    uint64_t r13_svc;
    uint64_t r14_abt;
    uint64_t r13_abt;
    uint64_t r14_und;
    uint64_t r13_und;
    uint64_t r8_fiq;
    uint64_t r9_fiq;
    uint64_t r10_fiq;
    uint64_t r11_fiq;
    uint64_t r12_fiq;
    uint64_t r13_fiq;
    uint64_t r14_fiq;
    uint64_t pc;
    uint64_t cpsr;
    uint64_t r13_mon;
    uint64_t r14_mon;
    uint64_t r14_hyp;
    uint64_t _reserved;
    uint64_t __reserved1;
    uint64_t __reserved2;
    uint64_t __reserved3;
    uint64_t __reserved4;
};

struct cpu_ctxt_regs
{
     struct cpu32_ctxt_regs_type cpu32_regs;
     struct cpu32_secure_ctxt_regs_type secure_contex;
};

struct wdt_ctx_info
{
      char *buf;
      unsigned long bufsize;
      unsigned long charend;
     // spinlock_t  buf_lock;
};
struct wdt_ctx_info wdt_ctx_s;

//modify by jch for update watch dog ramdump PR-802266
/*
int wdt_save_info(unsigned long cpu_data_addr){

	return  pstore_save_wdt_info(cpu_data_addr);
}
EXPORT_SYMBOL(wdt_save_info);
*/
//end modify by jch for update watch dog ramdump PR-802266

void pstore_wdt_init(void)
{
    wdt_ctx_s.bufsize = 16*1024UL;
	wdt_ctx_s.charend = 0;
    wdt_ctx_s.buf = kzalloc(wdt_ctx_s.bufsize, GFP_KERNEL);
 if (!wdt_ctx_s.buf ) {
		printk("WDT ctx dump buf allocation failed\n");
		goto out;
	 }
	 return;
out:
	kfree(wdt_ctx_s.buf);
	return;
}


int pstore_wdt_print(const char *fmt, ...)
{
	va_list args;
	int len = 0;
	char buf_line[256];
	//unsigned long flags;

	va_start(args, fmt);
	len = vsnprintf(buf_line, sizeof(buf_line) , fmt, args);
	va_end(args);

	if (!wdt_ctx_s.buf ) {
		printk("WDT ctx print buf is null!\n");
		return 0;
	}

	if(wdt_ctx_s.charend >= wdt_ctx_s.bufsize){
		printk("WDT:buf size is not enough!");
		return 0;
	}

	if (len > (wdt_ctx_s.bufsize -wdt_ctx_s.charend))
		len = wdt_ctx_s.bufsize -wdt_ctx_s.charend;

	//spin_lock_irqsave(&wdt_ctx_s.buf_lock, flags);	
	memcpy(wdt_ctx_s.buf + wdt_ctx_s.charend, buf_line, len);
	wdt_ctx_s.charend += len;
	//spin_unlock_irqrestore(&wdt_ctx_s.buf_lock, flags);		

	return len;
}

#define TCTWDT(fmt, args...) do {			\
	pstore_wdt_print(fmt, ##args);	\
} while (0)

void wdt_print_symbol(const char *fmt, uint64_t address)
{       
	char buffer[256];
	sprint_symbol(buffer,(unsigned long)address);
	TCTWDT(fmt, buffer);
}

static void wdt_show_regs(struct cpu_ctxt_regs *cpu_ctxt,
			const char *label)
{        
	TCTWDT("\n%s:\n", label);
	wdt_print_symbol("PC is at: %s ",cpu_ctxt->cpu32_regs.pc);
	TCTWDT("<0x%08llx>\n", cpu_ctxt->cpu32_regs.pc);
	wdt_print_symbol("LR is at: %s ",cpu_ctxt->cpu32_regs.r14_svc);
	TCTWDT("<0x%08llx>\n", cpu_ctxt->cpu32_regs.r14_svc);
    TCTWDT("\n%s regs:\n", label);
	TCTWDT("\t       r0 : 0x%08llx          r1 : 0x%08llx         r2 : 0x%08llx         r3 : 0x%08llx         r4 : 0x%08llx         r5 : 0x%08llx         r6 : 0x%08llx\n",
		cpu_ctxt->cpu32_regs.r0, cpu_ctxt->cpu32_regs.r1, cpu_ctxt->cpu32_regs.r2,cpu_ctxt->cpu32_regs.r3, cpu_ctxt->cpu32_regs.r4, cpu_ctxt->cpu32_regs.r5, cpu_ctxt->cpu32_regs.r6);
	TCTWDT("\t       r7 : 0x%08llx          r8 : 0x%08llx         r9 : 0x%08llx        r10 : 0x%08llx        r11 : 0x%08llx \n",
		cpu_ctxt->cpu32_regs.r7, cpu_ctxt->cpu32_regs.r8, cpu_ctxt->cpu32_regs.r9,	cpu_ctxt->cpu32_regs.r10,cpu_ctxt->cpu32_regs.r11);
	TCTWDT("\t      r12 : 0x%08llx     r13_usr : 0x%08llx    r14_usr : 0x%08llx    r13_hyp : 0x%08llx    r14_irq : 0x%08llx \n",
		cpu_ctxt->cpu32_regs.r12, cpu_ctxt->cpu32_regs.r13_usr,cpu_ctxt->cpu32_regs.r14_usr,cpu_ctxt->cpu32_regs.r13_hyp, cpu_ctxt->cpu32_regs.r14_irq);
	TCTWDT("\t  r13_irq : 0x%08llx     r14_svc : 0x%08llx    r13_svc : 0x%08llx    r14_abt : 0x%08llx    r13_abt : 0x%08llx\n",
		cpu_ctxt->cpu32_regs.r13_irq,cpu_ctxt->cpu32_regs.r14_svc,cpu_ctxt->cpu32_regs.r13_svc, cpu_ctxt->cpu32_regs.r14_abt, cpu_ctxt->cpu32_regs.r13_abt);
	TCTWDT("\t  r14_und : 0x%08llx     r13_und : 0x%08llx     r8_fiq : 0x%08llx     r9_fiq : 0x%08llx    r10_fiq : 0x%08llx\n",
		cpu_ctxt->cpu32_regs.r14_und,cpu_ctxt->cpu32_regs.r13_und, cpu_ctxt->cpu32_regs.r8_fiq, cpu_ctxt->cpu32_regs.r9_fiq,cpu_ctxt->cpu32_regs.r10_fiq);	
	TCTWDT("\t  r11_fiq : 0x%08llx     r12_fiq : 0x%08llx    r13_fiq : 0x%08llx    r14_fiq : 0x%08llx         pc : 0x%08llx\n",
		cpu_ctxt->cpu32_regs.r11_fiq, cpu_ctxt->cpu32_regs.r12_fiq, cpu_ctxt->cpu32_regs.r13_fiq,cpu_ctxt->cpu32_regs.r14_fiq,cpu_ctxt->cpu32_regs.pc);
	TCTWDT("\t     cpsr : 0x%08llx     r13_mon : 0x%08llx    r14_mon : 0x%08llx   reserved : 0x%08llx  reserved1 : 0x%08llx\n",
		cpu_ctxt->cpu32_regs.cpsr, cpu_ctxt->cpu32_regs.r13_mon,cpu_ctxt->cpu32_regs.r14_mon,cpu_ctxt->cpu32_regs._reserved, cpu_ctxt->cpu32_regs.__reserved1);		
	TCTWDT("\treserved2 : 0x%08llx   reserved3 : 0x%08llx  reserved4 : 0x%08llx\n",
		cpu_ctxt->cpu32_regs.__reserved2,cpu_ctxt->cpu32_regs.__reserved3, cpu_ctxt->cpu32_regs.__reserved4);
	TCTWDT("\n%s secure contex:\n", label);
	TCTWDT("\t       r0 : 0x%08llx          r1 : 0x%08llx         r2 : 0x%08llx         r3 : 0x%08llx         r4 : 0x%08llx         r5 : 0x%08llx         r6 : 0x%08llx\n",
		cpu_ctxt->secure_contex.r0, cpu_ctxt->secure_contex.r1, cpu_ctxt->secure_contex.r2,cpu_ctxt->secure_contex.r3, cpu_ctxt->secure_contex.r4, cpu_ctxt->secure_contex.r5, cpu_ctxt->secure_contex.r6);
	TCTWDT("\t       r7 : 0x%08llx          r8 : 0x%08llx         r9 : 0x%08llx        r10 : 0x%08llx        r11 : 0x%08llx \n",
		cpu_ctxt->secure_contex.r7, cpu_ctxt->secure_contex.r8, cpu_ctxt->secure_contex.r9,	cpu_ctxt->secure_contex.r10,cpu_ctxt->secure_contex.r11);
	TCTWDT("\t      r12 : 0x%08llx     r13_usr : 0x%08llx    r14_usr : 0x%08llx    r13_hyp : 0x%08llx    r14_irq : 0x%08llx \n",
		cpu_ctxt->secure_contex.r12, cpu_ctxt->secure_contex.r13_usr,cpu_ctxt->secure_contex.r14_usr,cpu_ctxt->secure_contex.r13_hyp, cpu_ctxt->secure_contex.r14_irq);
	TCTWDT("\t  r13_irq : 0x%08llx     r14_svc : 0x%08llx    r13_svc : 0x%08llx    r14_abt : 0x%08llx    r13_abt : 0x%08llx\n",
		cpu_ctxt->secure_contex.r13_irq,cpu_ctxt->secure_contex.r14_svc,cpu_ctxt->secure_contex.r13_svc, cpu_ctxt->secure_contex.r14_abt, cpu_ctxt->secure_contex.r13_abt);
	TCTWDT("\t  r14_und : 0x%08llx     r13_und : 0x%08llx     r8_fiq : 0x%08llx     r9_fiq : 0x%08llx    r10_fiq : 0x%08llx\n",
		cpu_ctxt->secure_contex.r14_und,cpu_ctxt->secure_contex.r13_und, cpu_ctxt->secure_contex.r8_fiq, cpu_ctxt->secure_contex.r9_fiq,cpu_ctxt->secure_contex.r10_fiq);	
	TCTWDT("\t  r11_fiq : 0x%08llx     r12_fiq : 0x%08llx    r13_fiq : 0x%08llx    r14_fiq : 0x%08llx         pc : 0x%08llx\n",
		cpu_ctxt->secure_contex.r11_fiq, cpu_ctxt->secure_contex.r12_fiq, cpu_ctxt->secure_contex.r13_fiq,cpu_ctxt->secure_contex.r14_fiq,cpu_ctxt->secure_contex.pc);
	TCTWDT("\t     cpsr : 0x%08llx     r13_mon : 0x%08llx    r14_mon : 0x%08llx   reserved : 0x%08llx  reserved1 : 0x%08llx\n",
		cpu_ctxt->secure_contex.cpsr, cpu_ctxt->secure_contex.r13_mon,cpu_ctxt->secure_contex.r14_mon,cpu_ctxt->secure_contex._reserved, cpu_ctxt->secure_contex.__reserved1);		
	TCTWDT("\treserved2 : 0x%08llx   reserved3 : 0x%08llx  reserved4 : 0x%08llx\n",
		cpu_ctxt->secure_contex.__reserved2,cpu_ctxt->secure_contex.__reserved3, cpu_ctxt->secure_contex.__reserved4);
}

static ssize_t last_cpu_knob_read(struct file *f, char __user *buf,
		size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, wdt_ctx_s.buf, wdt_ctx_s.charend);
}

static const struct file_operations last_cpu_fops = {
	.open	= simple_open,
	.read	= last_cpu_knob_read,	
};

void mk_last_cpu(void)
{      
	if (!wdt_ctx_s.buf )
		return;
	if (!proc_create("last_cpu", S_IRUSR, NULL,&last_cpu_fops)){
		printk("WDT: creat last_cpu  fail!\n");
		return;
	}
 
	return;
}

//modify by jch for update watch dog ramdump PR-802266
//void wdt_ctx_print( void )
void wdt_ctx_print(  unsigned long cpudata_addr )
//end modify by jch for update watch dog ramdump PR-802266
{	

    struct msm_dump_data *cpu_data;
	unsigned long addr;
	char label[64];	
	struct cpu_ctxt_regs *ctxt_regs;	
	const int cpu_count = get_core_count();
	int i;
	
    pstore_wdt_init();
      
	TCTWDT("************ WDT CPU Status Dump By TZ Start ************\n\n");

//modify by jch for update watch dog ramdump PR-802266
    //cpu_data = (struct msm_dump_data *)pstore_get_wdt_info();
    cpu_data = (struct msm_dump_data *)phys_to_virt(cpudata_addr);
//end modify by jch for update watch dog ramdump PR-802266
	if(!cpu_data){
		printk("WDT:TZ BSP dump cpu data might be not found!\n"); 
		TCTWDT("TZ BSP dump cpu data might be not founed!\n"); 
              goto out;
	}

	if(cpu_data->magic != TZBSP_DUMP_CTX_MAGIC || cpu_data->version!=TZBSP_DUMP_CTX_VERSION ){
		TCTWDT("Magic 0x%x version 0x%x doesn't match! No context will be parsed.\n",cpu_data->magic,cpu_data->version);
		printk("WDT:Magic 0x%x version 0x%x doesn't match! No context will be parsed.\n",cpu_data->magic,cpu_data->version);
              goto out;		
	}  
	printk("WDT:Parsing debug information Version: %d, Magic: 0x%x,reserved: 0x%x.\n",cpu_data->version,cpu_data->magic,cpu_data->reserved);
	if(!cpu_data->addr){
		printk("WDT:TZ BSP dump buffer might be not found!\n"); 
	       TCTWDT("TZ BSP dump buffer might be not found!\n"); 
              goto out;	
	} 
   	printk("WDT:cpu_data:Parsing debug information: 0x%08llx\n",cpu_data->addr);
	addr = (unsigned long)cpu_data->addr;
		ctxt_regs = (struct cpu_ctxt_regs *)phys_to_virt(addr);
   	if (!ctxt_regs) {
		 TCTWDT(" TZ BSP dump buffer might be mismatch. \n");	
		 goto out;
	}
	TCTWDT("Start to dump panic info:\n");

	//tct_smem_reason = 2; //add by shenghua.gong@tcl.com PR[825698]
	
	for (i = 0; i < cpu_count; i++) {
		snprintf(label, sizeof(label) - 1, "CPU%d", i);
		addr +=  i*MAX_CPU_CTX_PAGE_SIZE;
		ctxt_regs = (struct cpu_ctxt_regs *)phys_to_virt(addr);		
		wdt_show_regs(ctxt_regs, label);		
	}

out:
	TCTWDT("\n************ WDT CPU Status Dump By TZ End ************\n");
	mk_last_cpu();
	return;	
}
EXPORT_SYMBOL(wdt_ctx_print);
#endif //CONFIG_TCT_WATCHDOG_CTX_PRINT
//end add by jch for watch dog ramdump PR-802266
#endif

uint32_t msm_dump_table_version(void)
{
	return MSM_DUMP_TABLE_VERSION;
}
EXPORT_SYMBOL(msm_dump_table_version);

static int msm_dump_table_register(struct msm_dump_entry *entry)
{
	struct msm_dump_entry *e;
	struct msm_dump_table *table = memdump.table;

	if (!table || table->num_entries >= MAX_NUM_ENTRIES)
		return -EINVAL;

	e = &table->entries[table->num_entries];
	e->id = entry->id;
	e->type = MSM_DUMP_TYPE_TABLE;
	e->addr = entry->addr;
	table->num_entries++;

	dmac_flush_range(table, (void *)table + sizeof(struct msm_dump_table));
	return 0;
}

static struct msm_dump_table *msm_dump_get_table(enum msm_dump_table_ids id)
{
	struct msm_dump_table *table = memdump.table;
	int i;

	if (!table) {
		pr_err("mem dump base table does not exist\n");
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < MAX_NUM_ENTRIES; i++) {
		if (table->entries[i].id == id)
			break;
	}
	if (i == MAX_NUM_ENTRIES || !table->entries[i].addr) {
		pr_err("mem dump base table entry %d invalid\n", id);
		return ERR_PTR(-EINVAL);
	}

	/* Get the apps table pointer */
	table = phys_to_virt(table->entries[i].addr);

	return table;
}

int msm_dump_data_register(enum msm_dump_table_ids id,
			   struct msm_dump_entry *entry)
{
	struct msm_dump_entry *e;
	struct msm_dump_table *table;

	table = msm_dump_get_table(id);
	if (IS_ERR(table))
		return PTR_ERR(table);

	if (!table || table->num_entries >= MAX_NUM_ENTRIES)
		return -EINVAL;

	e = &table->entries[table->num_entries];
	e->id = entry->id;
	e->type = MSM_DUMP_TYPE_DATA;
	e->addr = entry->addr;
	table->num_entries++;

	dmac_flush_range(table, (void *)table + sizeof(struct msm_dump_table));
	return 0;
}
EXPORT_SYMBOL(msm_dump_data_register);

static int __init init_memory_dump(void)
{
	struct msm_dump_table *table;
	struct msm_dump_entry entry;
	struct device_node *np;
	void __iomem *imem_base;
	int ret;

	np = of_find_compatible_node(NULL, NULL,
				     "qcom,msm-imem-mem_dump_table");
	if (!np) {
		pr_err("mem dump base table DT node does not exist\n");
		return -ENODEV;
	}

	imem_base = of_iomap(np, 0);
	if (!imem_base) {
		pr_err("mem dump base table imem offset mapping failed\n");
		return -ENOMEM;
	}

	memdump.table = kzalloc(sizeof(struct msm_dump_table), GFP_KERNEL);
	if (!memdump.table) {
		pr_err("mem dump base table allocation failed\n");
		ret = -ENOMEM;
		goto err0;
	}
	memdump.table->version = MSM_DUMP_TABLE_VERSION;
	memdump.table_phys = virt_to_phys(memdump.table);
	writel_relaxed(memdump.table_phys, imem_base);
	/* Ensure write to imem_base is complete before unmapping */
	mb();
	pr_info("MSM Memory Dump base table set up\n");

	iounmap(imem_base);

	table = kzalloc(sizeof(struct msm_dump_table), GFP_KERNEL);
	if (!table) {
		pr_err("mem dump apps data table allocation failed\n");
		ret = -ENOMEM;
		goto err1;
	}
	table->version = MSM_DUMP_TABLE_VERSION;

	entry.id = MSM_DUMP_TABLE_APPS;
	entry.addr = virt_to_phys(table);
	ret = msm_dump_table_register(&entry);
	if (ret) {
		pr_info("mem dump apps data table register failed\n");
		goto err2;
	}
	pr_info("MSM Memory Dump apps data table set up\n");

	return 0;
err2:
	kfree(table);
err1:
	kfree(memdump.table);
	return ret;
err0:
	iounmap(imem_base);
	return ret;
}
early_initcall(init_memory_dump);

#ifdef CONFIG_MSM_DEBUG_LAR_UNLOCK
static int __init init_debug_lar_unlock(void)
{
	int ret;
	uint32_t argument = 0;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
		ret = scm_call(SCM_SVC_TZ, SCM_CMD_DEBUG_LAR_UNLOCK, &argument,
			       sizeof(argument), NULL, 0);
	else
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ,
				SCM_CMD_DEBUG_LAR_UNLOCK), &desc);
	if (ret)
		pr_err("Core Debug Lock unlock failed, ret: %d\n", ret);
	else
		pr_info("Core Debug Lock unlocked\n");

	return ret;
}
early_initcall(init_debug_lar_unlock);
#endif
