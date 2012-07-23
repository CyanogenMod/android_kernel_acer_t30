#include <linux/init.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/ram_log.h>
#include <linux/io.h>
#include <linux/uaccess.h>

struct ramlog_buffer {
	uint32_t    sig;
	uint32_t    index;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

#define RAMLOG_SIG (0x66063131) /* DBGC */

static char *ramlog_old_log[LOGCAT_TYPE_MAX];
static size_t ramlog_old_log_size[LOGCAT_TYPE_MAX];

static struct ramlog_buffer *ramlog_buffer[LOGCAT_TYPE_MAX];
static size_t ramlog_buffer_size[LOGCAT_TYPE_MAX];

static const char *last_logcat_file_name[LOGCAT_TYPE_MAX] = {
	"last_logcat",	/* LOGCAT_MAIN_SYSTEM_LOG */
	"last_radio",	/* LOGCAT_RADIO_LOG */
};

static void ramlog_update(const char *s,
					unsigned int count, size_t i)
{
	struct ramlog_buffer *buffer = ramlog_buffer[i];
	memcpy(buffer->data + buffer->start, s, count);
}

static void ramlog_write(const char *s,
					unsigned int count, size_t i)
{
	int rem;
	struct ramlog_buffer *buffer = ramlog_buffer[i];

	if (!buffer) return;

	if (count > ramlog_buffer_size[i]) {
		s += count - ramlog_buffer_size[i];
		count = ramlog_buffer_size[i];
	}
	rem = ramlog_buffer_size[i] - buffer->start;
	if (rem < count) {
		ramlog_update(s, rem, i);
		s += rem;
		count -= rem;
		buffer->start = 0;
		buffer->size = ramlog_buffer_size[i];
	}
	ramlog_update(s, count, i);

	buffer->start += count;
	if (buffer->size < ramlog_buffer_size[i])
		buffer->size += count;
}

static void __init
ramlog_save_old(struct ramlog_buffer *buffer, char *dest)
{
	size_t old_log_size = buffer->size;
	size_t i = buffer->index;

	if (dest == NULL) {
		dest = kmalloc(old_log_size, GFP_KERNEL);
		if (dest == NULL) {
			printk(KERN_ERR
			       "ramlog: failed to allocate buffer\n");
			return;
		}
	}

	ramlog_old_log[i] = dest;
	ramlog_old_log_size[i] = old_log_size;
	memcpy(ramlog_old_log[i],
	       &buffer->data[buffer->start], buffer->size - buffer->start);
	memcpy(ramlog_old_log[i] + buffer->size - buffer->start,
	       &buffer->data[0], buffer->start);
}

static int __init ramlog_init(char *buffer,
					size_t buffer_size, char *old_buf)
{
	struct ramlog_buffer *buf;
	size_t limit = buffer_size/LOGCAT_TYPE_MAX;
	size_t i;

	for (i = 0; i < LOGCAT_TYPE_MAX; i++) {
		ramlog_buffer[i] = (struct ramlog_buffer *)buffer;
		ramlog_buffer_size[i] = limit - sizeof(struct ramlog_buffer);
		buffer += limit;

		if (ramlog_buffer_size[i] > limit) {
			pr_err("ramlog: buffer[%d] %p, invalid size %zu, "
				"datasize %zu\n", i, ramlog_buffer[i], limit,
				ramlog_buffer_size[i]);
			return 0;
		}

		buf = ramlog_buffer[i];

		if (buf->sig == RAMLOG_SIG) {
			if (buf->size > ramlog_buffer_size[i]
				|| buf->start > buf->size)
				printk(KERN_INFO "ramlog: found existing invalid "
					"buffer, size %d, start %d\n",
					buf->size, buf->start);
			else {
				printk(KERN_INFO "ramlog: found existing buffer, "
					"size %d, start %d\n",
					buf->size, buf->start);
				ramlog_save_old(buf, old_buf);
			}
		} else {
			printk(KERN_INFO "ramlog: no valid data in buffer "
				"(sig = 0x%08x)\n", buf->sig);
		}

		buf->sig = RAMLOG_SIG;
		buf->index = i;
		buf->start = 0;
		buf->size = 0;
	}
	return 0;
}

static int ramlog_driver_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;

	if (res == NULL || pdev->num_resources != 1 ||
	    !(res->flags & IORESOURCE_MEM) || !res->start) {
		printk(KERN_ERR "ramlog: invalid resource, %p %d flags "
		       "%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}
	buffer_size = res->end - res->start + 1;
	start = res->start;
	printk(KERN_INFO "ramlog: got buffer at %zx, size %zx\n",
	       start, buffer_size);
	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "ramlog: failed to map memory\n");
		return -ENOMEM;
	}

	return ramlog_init(buffer, buffer_size, NULL);
}

static struct platform_driver ramlog_driver = {
	.probe = ramlog_driver_probe,
	.driver		= {
		.name	= "acer_ramlog",
	},
};

static int __init ramlog_module_init(void)
{
	int err;
	err = platform_driver_register(&ramlog_driver);
	return err;
}

static ssize_t ramlog_logcat_read_old(struct file *file,
					char __user *buf, size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	size_t i = LOGCAT_MAIN_SYSTEM_LOG;
	ssize_t count;

	if (pos >= ramlog_old_log_size[i])
		return 0;

	count = min(len, (size_t)(ramlog_old_log_size[i] - pos));
	if (copy_to_user(buf, ramlog_old_log[i] + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static ssize_t ramlog_radio_read_old(struct file *file,
					char __user *buf, size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	size_t i = LOGCAT_RADIO_LOG;
	ssize_t count;

	if (pos >= ramlog_old_log_size[i])
		return 0;

	count = min(len, (size_t)(ramlog_old_log_size[i] - pos));
	if (copy_to_user(buf, ramlog_old_log[i] + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations ramlog_logcat_fops = {
	.owner = THIS_MODULE,
	.read = ramlog_logcat_read_old,
};

static const struct file_operations ramlog_radio_fops = {
	.owner = THIS_MODULE,
	.read = ramlog_radio_read_old,
};

static struct {
	const struct file_operations *proc_fops;
} ramlog_entry[] = {
	{ &ramlog_logcat_fops },
	{ &ramlog_radio_fops },
};

int ramlog_register(struct ramlog_file_ops *ops)
{
	int ret = 0;
	if (ops->index >= LOGCAT_TYPE_MAX) {
		pr_err("ramlog: invalid logcat type\n");
		ret = -ENODEV;
	}
	ops->write = ramlog_write;
	return ret;
}
EXPORT_SYMBOL(ramlog_register);

static int __init ramlog_late_init(void)
{
	struct proc_dir_entry *entry;
	size_t i;

	for (i = 0; i < LOGCAT_TYPE_MAX; i++) {
		if (ramlog_old_log[i] == NULL)
			continue;
		entry = create_proc_entry(
			last_logcat_file_name[i], S_IFREG | S_IRUGO, NULL);
		if (!entry) {
			printk(KERN_ERR "ramlog: failed to create proc/%s entry\n",
							last_logcat_file_name[i]);
			kfree(ramlog_old_log[i]);
			ramlog_old_log[i] = NULL;
			continue;
		}
		entry->proc_fops = ramlog_entry[i].proc_fops;
		entry->size = ramlog_old_log_size[i];
	}
	return 0;
}

postcore_initcall(ramlog_module_init);
late_initcall(ramlog_late_init);
