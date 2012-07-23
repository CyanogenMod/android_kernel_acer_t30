#ifndef __LINUX_MISC_CMD_H
#define __LINUX_MISC_CMD_H

#if defined(CONFIG_ARCH_ACER_T20) || defined(CONFIG_ARCH_ACER_T30)
#define MSC_PATH "/dev/block/mmcblk0p5"

#if defined(CONFIG_ARCH_ACER_T20)
#define USE_OLD_MISC_CMD 1
#endif

#ifdef USE_OLD_MISC_CMD
typedef struct{
	unsigned char command[12];
	unsigned char debug_switch;
	unsigned char display_debug;
	unsigned char size;
} BootloaderMessage;
#else
typedef struct{
	char command[32];
	char status[32];
#ifdef CONFIG_ACER_RAM_LOG
	unsigned ramlog_switch;
#endif
	unsigned debug_switch;
} BootloaderMessage;
#endif
#endif

#endif /* __LINUX_MISC_CMD_H */
