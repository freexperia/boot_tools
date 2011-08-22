/*
 * splload.c - splload
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <mach/msm_iomap.h>
#include <asm/mach/map.h>

#define IMAGE_LOADER_SIZE   0x000ff000
#define IMAGE_PARAM_SIZE    0x00001000
#define IMAGE_BOOT_ADDR     0x03200000
#define IMAGE_BOOT_IMG_SIZE 0x00700000
#define IMAGE_SPL_SIZE      (IMAGE_LOADER_SIZE + IMAGE_PARAM_SIZE)
#define IMAGE_SPL_ADDR      0x30000000

#define BOOTPARAM_SIZE      0x00000400
#define SERIALNO_SIZE       0x00000020

extern void setup_mm_for_reboot(char mode);

static unsigned char* splmem = 0;
static unsigned char* __iomem spladdr = 0;
static unsigned char* __iomem mddiaddr = 0;
static unsigned char* __iomem bootimgaddr = 0;
static off_t spl_off = 0;
static off_t bootimg_off = 0;
static int spl_image_stored = 0;
static int boot_image_stored = 0;
static int s_spladdr = IMAGE_SPL_ADDR; // copy dest kernel image
static int s_splsize = IMAGE_SPL_SIZE; // fastboot and boot.img
static int s_machine_type = 1009000; //ES209RA
static char s_bootparam[BOOTPARAM_SIZE] = {0};
static char s_serialno[SERIALNO_SIZE] = {'S', '1', '2', '3', '4', '5', '\0'};

module_param_named(spladdr, s_spladdr, int, 0644);
module_param_named(splsize, s_splsize, int, 0644);

module_param_named(machine_type, s_machine_type, int, 0644);
module_param_string(bootparam, s_bootparam, BOOTPARAM_SIZE, 0644);
module_param_string(serialno, s_serialno, SERIALNO_SIZE, 0644);

typedef struct st_loader_param {
	int version;
	int param_size;
	unsigned int mddiaddr;
	unsigned int bootimgaddr;
	unsigned int bootimgsize;
	int machine_type;
	char bootparam[BOOTPARAM_SIZE];
	char serialno[SERIALNO_SIZE];
}loader_param;

enum{
	IMAGE_TYPE_LOADER = 0,
	IMAGE_TYPE_BOOT_IMG
};

/*
--------------------------
   SPL(usbloader)
--------------------------
   parameter for loader
--------------------------
*/


static int proc_splimage_show(char *page, char **start, off_t off,
                          int count, int *eof, void *data)
{
//printk("page=%p start=%p off=%08x, count=%08x eof=%d data=%p\n",
//	page, *start, (unsigned int)off, count, *eof, data);
	int image_type = (int)data;
	int image_size = 0;
	unsigned char* image_addr = NULL;

	if(image_type == IMAGE_TYPE_LOADER){
		image_addr = splmem;
		image_size = IMAGE_LOADER_SIZE;
	}else if(image_type == IMAGE_TYPE_BOOT_IMG){
		image_addr = bootimgaddr;
		image_size = IMAGE_BOOT_IMG_SIZE;
	}
	if(off > image_size){
		*eof=1;
		return 0;
	}
	if(off + count >= image_size){
		count = image_size - off;
	}
	memcpy(page, image_addr + off, count);
	*start = page;
	spl_off = 0;
	bootimg_off = 0;
	return count;
}

static int proc_splimage_store(struct file *file,
                             const char *buffer,
                             unsigned long count,
                             void *data)
{
	int image_size = 0;
	int image_off = 0;
	unsigned char* image_addr = NULL;
	int image_type = (int)data;

//printk("off=%08x file=%p buffer=%p count=%08lx data=%p\n",
//	(unsigned int)spl_off, file, buffer, count, data);

	if(image_type == IMAGE_TYPE_LOADER){
		image_size = IMAGE_LOADER_SIZE;
		image_addr = splmem;
		image_off = spl_off;
		spl_image_stored = 1;
	}else if(image_type == IMAGE_TYPE_BOOT_IMG){
		image_addr = bootimgaddr;
		image_size = IMAGE_BOOT_IMG_SIZE;
		image_off = bootimg_off;
		boot_image_stored = 1;
	}

	if(image_off > image_size){
		return count;
	}
	if(image_off + count >= image_size){
		count = image_size - image_off;
	}

	memcpy(image_addr+image_off, buffer, count);

	if(image_type == IMAGE_TYPE_LOADER){
		spl_off += count;
	}else if(image_type == IMAGE_TYPE_BOOT_IMG){
		bootimg_off += count;
	}
	return count;
}

static void memdump(const unsigned char* buf, int size)
{
#if 0
	int i;
	for(i=0; i<size; i++){
		if(i % 16 == 0) printk(KERN_INFO "%04x:", i);
		printk(" %02x", buf[i]);
		if(i % 16 == 15) printk("\n");
	}
	printk("\n");
#endif
}

static int proc_splboot_store(struct file *file,
                             const char *buffer,
                             unsigned long count,
                             void *data)
{
	loader_param* param = (loader_param*)(splmem+IMAGE_LOADER_SIZE);
	if(!spl_image_stored){
		printk(KERN_EMERG "Need store image.\n");
		return 0;
	}
	printk(KERN_EMERG "Starting new spl\n");

	param->version = 0;
	param->param_size = sizeof(*param);
	param->mddiaddr = (unsigned int)mddiaddr;
	param->machine_type = s_machine_type;
	strncpy(param->bootparam, s_bootparam, BOOTPARAM_SIZE);
	strncpy(param->serialno, s_serialno, SERIALNO_SIZE);
	if(boot_image_stored){
		param->bootimgaddr = IMAGE_BOOT_ADDR;
		param->bootimgsize = bootimg_off;
	}
	memcpy(spladdr, splmem, s_splsize);

	flush_icache_range((unsigned long) spladdr,
			   (unsigned long) spladdr + s_splsize);
	printk(KERN_INFO "Bye!\n");
	setup_mm_for_reboot(0);
	memdump(spladdr, 0x100);
	printk(KERN_INFO "spladdr=%p s_spladdr=%08x\n", spladdr, s_spladdr);
	spladdr = (void*)s_spladdr;
	memdump(spladdr, 0x100);
	cpu_reset(s_spladdr);
	return 0;
}

static struct proc_dir_entry *splboot_dir;

static void splboot_free(void)
{
	if (spladdr)
		iounmap(spladdr);

	if(mddiaddr)
		iounmap(mddiaddr);

	if(bootimgaddr)
		iounmap(bootimgaddr);

	if(splmem)
		kfree(splmem);

	if(splboot_dir) {
		remove_proc_entry("image", splboot_dir);
		remove_proc_entry("boot", splboot_dir);
		remove_proc_entry("boot_img", splboot_dir);
	}
	remove_proc_entry("splboot", NULL);
}

static int __init splboot_init(void)
{
	struct proc_dir_entry *entry;
	splboot_dir = proc_mkdir("splboot", NULL);

	splmem = kzalloc(s_splsize, GFP_KERNEL);
	if(!splmem)
		goto fail;
	spladdr = ioremap(s_spladdr, s_splsize);
	if (!spladdr)
		goto fail;

	mddiaddr = ioremap(0xAA600000, 0x0001000);
	if (!mddiaddr)
		goto fail;

	bootimgaddr = ioremap(IMAGE_BOOT_ADDR, IMAGE_BOOT_IMG_SIZE);
	if (!bootimgaddr)
		goto fail;

printk("mddiaddr=%p\n", mddiaddr);
printk("bootimgaddr=%p\n", bootimgaddr);
	entry = create_proc_entry("image", S_IFREG | S_IRUGO | S_IWUGO, splboot_dir);
	entry->data = (void*)IMAGE_TYPE_LOADER;
	entry->read_proc = proc_splimage_show;
	entry->write_proc = proc_splimage_store;
	entry->owner = THIS_MODULE;

	entry = create_proc_entry("boot_img", S_IFREG | S_IRUGO | S_IWUGO, splboot_dir);
	entry->data = (void*)IMAGE_TYPE_BOOT_IMG;
	entry->read_proc = proc_splimage_show;
	entry->write_proc = proc_splimage_store;
	entry->owner = THIS_MODULE;

	entry = create_proc_entry("boot", S_IFREG | S_IRUGO | S_IWUGO, splboot_dir);
	entry->data = NULL;
	entry->write_proc = proc_splboot_store;
	entry->owner = THIS_MODULE;

	spl_image_stored = 0;

	return 0;
fail:
	splboot_free();
	return -ENOMEM;
}
module_init(splboot_init)

static void __exit splboot_exit(void)
{
	splboot_free();
}
module_exit(splboot_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("execute splload image");

