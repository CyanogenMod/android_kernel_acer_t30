#ifndef _LINUX_RAM_LOG_H
#define _LINUX_RAM_LOG_H

typedef enum {
	LOGCAT_MAIN_SYSTEM_LOG,
	LOGCAT_RADIO_LOG,
	LOGCAT_TYPE_MAX
} logcat_type_t;

struct ramlog_file_ops {
	size_t index;
	void (*write)(const char *, unsigned int, size_t);
};

int ramlog_register(struct ramlog_file_ops *);

#endif /* _LINUX_RAM_LOG_H */
