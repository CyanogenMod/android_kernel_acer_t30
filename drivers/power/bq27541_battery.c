/*
 * bq27541 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/bq27541.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#if defined(CONFIG_ARCH_ACER_T30)
#include "../../arch/arm/mach-tegra/board-acer-t30.h"
extern int acer_board_id;
extern int acer_board_type;
extern bool throttle_start;
#endif

#define RELEASED_DATE			"2012/07/17"
#define DRIVER_VERSION			"1.6.7"

#ifdef CONFIG_BATTERY_BQ27541

#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_TTE		0x16
#define BQ27541_REG_TTF		0x18
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOC		0x2c
#define BQ27541_REG_RSOC		0x0B

#define BQ27541_CNTL			0x00
#define BQ27541_ATRATE			0x02
#define BQ27541_ENERGY_AVAIL		0x22
#define BQ27541_POWER_AVG		0x24
#define BQ27541_CYCLE_COUNT		0x2a
#define BQ27541_DESIGN_CAPACITY	0x3c

#define BQ27541_HIBERNATE_I	0x4b
#define BQ27541_HIBERNATE_V	0x4d

/* Control status bit for sealed/full access sealed */
#define BQ27541_CNTL_SS		BIT(13)
#define BQ27541_CNTL_FAS		BIT(14)

/* FLAGS register bit definition*/
#define BQ27541_FLAG_DSG		BIT(0)
#define BQ27541_FLAG_SOCF		BIT(1)
#define BQ27541_FLAG_FC		BIT(9)
#define BQ27541_FLAG_CHGS		BIT(8)
#define BQ27541_FLAG_OTC		BIT(15)

/* Control register sub-commands */
#define BQ27541_CNTL_DEVICE_TYPE	0x0001
#define BQ27541_CNTL_FW_VERSION	0x0002
#define BQ27541_CNTL_HW_VERSION	0x0003
#define BQ27541_CNTL_SET_SLEEP		0x0013
#define BQ27541_CNTL_CLEAR_SLEEP	0x0014
#define BQ27541_CNTL_SET_SEALED	0x0020
#define BQ27541_CNTL_FULL_RESET	0x0041

/* Define battery poll /irq period */
#define BATTERY_POLL_PERIOD		30000
#define BATTERY_FAST_POLL_PERIOD	500

/* Define AC_OUT delay time */
#define DEBOUNCE	80

/* Define low temperature threshold */
#define REPORT_NORMAL_VAL		0
#define TEMP_UNDER_ZERO		1
#define TEMP_UNDER_NAT_TEN		2
#define RSOC_NOT_QUALIFY		3

/* Define charger report signal */
#define ADAPTER_PLUG_IN		1
#define ADAPTER_PULL_OUT		0

/* Define charger related GPIO */
#define BATT_LEARN	TEGRA_GPIO_PX6
#define CHARGER_STAT	TEGRA_GPIO_PJ2
#define CHARGING_FULL	TEGRA_GPIO_PW5

static struct wake_lock ac_wake_lock;
struct i2c_client *bat_client;
bool AC_IN = false;
int design_capacity = 0;
int cpu_temp = 0;
int bat_temp = 0;
int Capacity = 0;
int old_rsoc = 0;
int counter = 0;
int adapter = 0;

extern tegra3_cpu_temp_query(void);

struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di);
};

struct bq27541_device_info {
	struct device			*dev;
	struct bq27541_access_methods	*bus;
	struct power_supply		bat;
	struct power_supply		ac;
	struct timer_list		battery_poll_timer;
	struct i2c_client		*client;
	struct bq27541_platform_data	*plat_data;
	struct delayed_work		work;
	struct mutex		lock;
	int				id;
	int				irq;
};

static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};


static int bat_i2c_read(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}


/*
 * Create Sysfs debug console.
 * File path: /sys/BatControl
 */
static struct kobject *bq27541_dbg_kobj;

#define debug_attr(_name) \
	static struct kobj_attribute _name##_attr = { \
	.attr = { \
	.name = __stringify(_name), \
	.mode = 0644, \
	}, \
	.show = _name##_show, \
	}


static ssize_t Firmware_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_byte_data(bat_client, BQ27541_CNTL_FW_VERSION);
	msleep(10);
	s += sprintf(s, "%x\n", ret);
	return (s - buf);
}

static ssize_t Charge_Full_Design_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_DESIGN_CAPACITY);
	msleep(10);
	s += sprintf(s, "%d mAh\n", ret);
	return (s - buf);
}

static ssize_t Vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_byte_data(bat_client, BQ27541_CNTL_HW_VERSION);
	msleep(10);
	if(ret == 1)
		s += sprintf(s, "Sanyo\n");
	else
		s += sprintf(s, "LG\n");
	return (s - buf);
}

static ssize_t Health_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_FLAGS);
	msleep(10);

	if (ret & BQ27541_FLAG_SOCF)
		s += sprintf(s, "DEAD\n");
	else if (ret & BQ27541_FLAG_OTC)
		s += sprintf(s, "OVERHEAT\n");
	else
		s += sprintf(s, "GOOD\n");

	msleep(100);
	return (s - buf);
}

static ssize_t Temperature_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_TEMP);
	msleep(10);
	s += sprintf(s, "%d\n", ret - 2731 );

	return (s - buf);
}

static ssize_t Voltage_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_VOLT);
	msleep(10);
	s += sprintf(s, "%dmV\n", ret);

	return (s - buf);
}

static ssize_t Current_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_AI);
	msleep(10);
	s += sprintf(s, "%dmA\n", ret * 1000);

	return (s - buf);
}
static ssize_t Capacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_SOC);
	msleep(10);
	s += sprintf(s, "%d\n", ret);

	return (s - buf);
}

static ssize_t Status_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_FLAGS);
	msleep(10);

	if (ret & BQ27541_FLAG_DSG)
		s += sprintf(s, "DISCHARGING\n");
	else
		s += sprintf(s, "CHARGING\n");

	if (ret & BQ27541_FLAG_FC)
		s += sprintf(s, "FULL\n");

	return (s - buf);
}

static ssize_t tSealStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	int ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_CNTL);
	msleep(10);
	s += sprintf(s, "FAS = %x , SS = %x\n",
		(int)(ret & BQ27541_CNTL_FAS), (int)(ret & BQ27541_CNTL_SS));
	msleep(10);
	ret = i2c_smbus_read_word_data(bat_client, BQ27541_CNTL_FULL_RESET);
	s += sprintf(s, "Reset status = %d\n", ret);
	msleep(10);

	return (s - buf);
}

static ssize_t tGaugeReset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	/* write PASSWARD registry procedure : 04143672 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
	s += sprintf(s, "write 1st pw = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
	s += sprintf(s, "write 2nd pw = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_FULL_RESET);
	s += sprintf(s, "write resest registry = %d\n", ret);
	msleep(10);

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_CNTL_FULL_RESET);
	s += sprintf(s, "Reset status = %d\n", ret);
	msleep(10);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
	s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
	msleep(10);

	return (s - buf);
}

static ssize_t tChargerReset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;

	tegra_gpio_enable(BATT_LEARN);
	gpio_request(BATT_LEARN, "batt_learn");

	s += sprintf(s, "BATT_LEARN = %d\n", gpio_get_value(BATT_LEARN));
	gpio_direction_output(BATT_LEARN, 1);
	msleep(500);
	s += sprintf(s, "BATT_LEARN = %d\n", gpio_get_value(BATT_LEARN));
	gpio_direction_output(BATT_LEARN, 0);
	s += sprintf(s, "BATT_LEARN = %d\n", gpio_get_value(BATT_LEARN));

	return (s - buf);
}

static ssize_t tCheckSYHI_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret, hi, hv;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x40);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x16);
	s += sprintf(s, "write SY pw1 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x03);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x63);
	s += sprintf(s, "write SY pw2 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x53);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	ret = i2c_smbus_read_byte_data(bat_client, 0x40);
	s += sprintf(s, "Chem ID = %x\n", ret);
	msleep(50);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
	msleep(50);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
	s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
	msleep(50);

	return (s - buf);
}

static ssize_t tCheckLGHI_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret, hi, hv;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
	s += sprintf(s, "write LG pw1 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
	s += sprintf(s, "write LG pw2 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x38);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	ret = i2c_smbus_read_byte_data(bat_client, 0x44);
	s += sprintf(s, "FIRMWARE = %x\n", ret);
	msleep(50);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
	msleep(50);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
	s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
	msleep(50);

	return (s - buf);
}


static ssize_t tDisableHibernate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret, BDC, DFC, DFB, hi, hv;
	int sum = 0;
	int cheksum = 0;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x40);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x16);
	s += sprintf(s, "write SY pw1 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x03);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x63);
	s += sprintf(s, "write SY pw2 = %d\n", ret);
	msleep(1000);

	BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", BDC);
	msleep(200);
	DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", DFC);
	msleep(200);
	DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", DFB);

	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
	msleep(50);

	if(BDC < 0 || DFC < 0){
		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		s += sprintf(s, "clean SY pw1 = %d\n", ret);
		msleep(1000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		s += sprintf(s, "clean SY pw2 = %d\n", ret);
		msleep(1000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
		s += sprintf(s, "write LG pw1 = %d\n", ret);
		msleep(1000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
		s += sprintf(s, "write LG pw2 = %d\n", ret);
		msleep(1000);

		BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
		s += sprintf(s, "0x61 = %d\n", BDC);
		msleep(200);
		DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
		s += sprintf(s, "0x3e = %d\n", DFC);
		msleep(200);
		DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
		s += sprintf(s, "0x3f = %d\n", DFB);

		hi = i2c_smbus_read_word_data(bat_client, 0x4b);
		s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
		msleep(50);
		hv = i2c_smbus_read_word_data(bat_client, 0x4d);
		s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
		msleep(50);
	}

	if ((hi == 0) && (hv == 0))
	{
		s += sprintf(s, "Hibernate has been set!\n");
		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
		msleep(50);
	}
	else
	{
		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
		s += sprintf(s, "0x61 = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
		s += sprintf(s, "0x3e = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
		s += sprintf(s, "0x3f = %d\n", ret);

		ret = i2c_smbus_read_byte_data(bat_client, 0x40);
		s += sprintf(s, "=========== 0x40 = %x =========\n", ret);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x41);
		s += sprintf(s, "=========== 0x41 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x42);
		s += sprintf(s, "=========== 0x42 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x43);
		s += sprintf(s, "=========== 0x43 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x44);
		s += sprintf(s, "=========== 0x44 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x45);
		s += sprintf(s, "=========== 0x45 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x46);
		s += sprintf(s, "=========== 0x46 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x47);
		s += sprintf(s, "=========== 0x47 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x48);
		s += sprintf(s, "=========== 0x48 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x49);
		s += sprintf(s, "=========== 0x49 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4a);
		s += sprintf(s, "=========== 0x4a = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4b);
		s += sprintf(s, "=========== 0x4b = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4c);
		s += sprintf(s, "=========== 0x4c = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4d);
		s += sprintf(s, "=========== 0x4d = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4e);
		s += sprintf(s, "=========== 0x4e = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4f);
		s += sprintf(s, "=========== 0x4f = %x =========\n", ret);
		msleep(50);
		sum += ret;

		ret = i2c_smbus_read_byte_data(bat_client, 0x50);
		s += sprintf(s, "=========== 0x50 = %x =========\n", ret);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x51);
		s += sprintf(s, "=========== 0x51 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x52);
		s += sprintf(s, "=========== 0x52 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x53);
		s += sprintf(s, "=========== 0x53 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x54);
		s += sprintf(s, "=========== 0x54 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x55);
		s += sprintf(s, "=========== 0x55 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x56);
		s += sprintf(s, "=========== 0x56 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x57);
		s += sprintf(s, "=========== 0x57 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x58);
		s += sprintf(s, "=========== 0x58 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x59);
		s += sprintf(s, "=========== 0x59 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5a);
		s += sprintf(s, "=========== 0x5a = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5b);
		s += sprintf(s, "=========== 0x5b = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5c);
		s += sprintf(s, "=========== 0x5c = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5d);
		s += sprintf(s, "=========== 0x5d = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5e);
		s += sprintf(s, "=========== 0x5e = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5f);
		s += sprintf(s, "=========== 0x5f = %x =========\n", ret);
		msleep(50);
		sum += ret;
		s += sprintf(s, "=========== sum = %x =========\n", sum);
		ret = 255 - (sum % 256);
		cheksum = ret;
		s += sprintf(s, "=========== CHECKSUM = %x =========\n", ret);

		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
		s += sprintf(s, "0x61 = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
		s += sprintf(s, "0x3e = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
		s += sprintf(s, "0x3f = %d\n", ret);

		ret = i2c_smbus_write_byte_data(bat_client, 0x4b, 0x00);
		s += sprintf(s, "Clear Hibernate I (0x4b) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4c, 0x00);
		s += sprintf(s, "Clear Hibernate I (0x4c) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4d, 0x00);
		s += sprintf(s, "Clear Hibernate V (0x4d) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4e, 0x00);
		s += sprintf(s, "Clear Hibernate V (0x4e) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x60, cheksum);
		s += sprintf(s, "Checksum (0x60) = %x,\n", cheksum);
		msleep(200);

		ret = i2c_smbus_read_word_data(bat_client, 0x4b);
		s += sprintf(s, "Hibernate I (0x4b) = %x\n", ret);
		msleep(50);
		ret = i2c_smbus_read_word_data(bat_client, 0x4d);
		s += sprintf(s, "Hibernate V (0x4d) = %x\n", ret);
		msleep(50);

		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
		msleep(50);

	}
	return (s - buf);
}


debug_attr(Firmware);
debug_attr(Charge_Full_Design);
debug_attr(Vendor);
debug_attr(Health);
debug_attr(Temperature);
debug_attr(Voltage);
debug_attr(Current);
debug_attr(Capacity);
debug_attr(Status);
debug_attr(tSealStatus);
debug_attr(tGaugeReset);
debug_attr(tChargerReset);
debug_attr(tDisableHibernate);
debug_attr(tCheckSYHI);
debug_attr(tCheckLGHI);



static struct attribute * g[] = {
	&Firmware_attr.attr,
	&Charge_Full_Design_attr.attr,
	&Vendor_attr.attr,
	&Health_attr.attr,
	&Temperature_attr.attr,
	&Voltage_attr.attr,
	&Current_attr.attr,
	&Capacity_attr.attr,
	&Status_attr.attr,
	&tSealStatus_attr.attr,
	&tGaugeReset_attr.attr,
	&tChargerReset_attr.attr,
	&tDisableHibernate_attr.attr,
	&tCheckSYHI_attr.attr,
	&tCheckLGHI_attr.attr,
	NULL,
};

static struct attribute_group dbg_attr_group =
{
	.attrs = g,
};

static void bq27541_disable_hibernate(void)
{
	s32 ret, BDC, DFC, DFB, hi, hv;
	int sum = 0;
	int cheksum = 0;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x40);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x16);
	printk(KERN_INFO "%s -- write SY pw1 = %d\n",__func__ ,ret);
	msleep(2000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x03);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x63);
	printk(KERN_INFO "%s -- write SY pw2 = %d\n",__func__ ,ret);
	msleep(2000);

	/* write data lash block procedure */
	BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
	printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,ret);
	msleep(500);
	DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
	printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,ret);
	msleep(500);
	DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
	printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,ret);
	msleep(500);

	/* read Hibernate I&V status */
	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	printk(KERN_INFO "%s -- Hibernate I (0x4b) = %d\n",__func__ ,hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	printk(KERN_INFO "%s -- Hibernate V (0x4d) = %d\n",__func__ ,hv);
	msleep(50);

	if(BDC < 0 || DFC < 0){
		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		printk(KERN_INFO "%s -- clean SY pw1 = %d\n",__func__ ,ret);
		msleep(2000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		printk(KERN_INFO "%s -- clean SY pw2 = %d\n",__func__ ,ret);
		msleep(2000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
		printk(KERN_INFO "%s -- write LG pw1 = %d\n",__func__ ,ret);
		msleep(2000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
		printk(KERN_INFO "%s -- write LG pw2 = %d\n",__func__ ,ret);
		msleep(2000);

		/* write data lash block procedure */
		BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
		printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,BDC);
		msleep(500);
		DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
		printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,DFC);
		msleep(500);
		DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
		printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,DFB);
		msleep(500);

		/* read Hibernate I&V status */
		hi = i2c_smbus_read_word_data(bat_client, 0x4b);
		printk(KERN_INFO "%s -- Hibernate I (0x4b) = %x\n",__func__ ,hi);
		msleep(50);
		hv = i2c_smbus_read_word_data(bat_client, 0x4d);
		printk(KERN_INFO "%s -- Hibernate V (0x4d) = %x\n",__func__ ,hv);
		msleep(50);
	}

	if ((hi == 0) && (hv == 0))
	{
		printk(KERN_INFO "%s -- Hibernate has been set!\n",__func__ );
		/* set bq27541 to SEALED mode */
		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		printk(KERN_INFO "%s -- Change to Sealed Mode = %d\n",__func__ ,ret);
		msleep(50);
	}
	else
	{
		/* write data lash block procedure */
		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
		printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
		printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
		printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,ret);
		msleep(200);

		/* calculate data block checksum */
		ret = i2c_smbus_read_byte_data(bat_client, 0x40);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x41);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x42);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x43);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x44);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x45);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x46);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x47);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x48);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x49);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4a);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4b);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4c);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4d);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4e);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4f);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x50);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x51);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x52);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x53);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x54);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x55);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x56);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x57);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x58);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x59);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5a);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5b);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5c);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5d);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5e);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5f);
		msleep(50);
		sum += ret;
		cheksum = 255 - (sum % 256);
		printk(KERN_INFO "%s -- CHECKSUM = %x\n",__func__ ,ret);

		/* write data lash block procedure */
		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
		printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
		printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
		printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,ret);
		msleep(200);

		/* clear Hibernate I&V procedure */
		ret = i2c_smbus_write_byte_data(bat_client, 0x4b, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate I (0x4b) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4c, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate I (0x4c) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4d, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate V (0x4d) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4e, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate V (0x4e) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x60, cheksum);
		printk(KERN_INFO "%s -- Checksum (0x60) = %x\n",__func__ ,cheksum);
		msleep(200);

		/* confirm Hibernate I&V status : 0 */
		ret = i2c_smbus_read_word_data(bat_client, 0x4b);
		printk(KERN_INFO "%s -- Hibernate I (0x4b) = %x\n",__func__ ,ret);
		msleep(50);
		ret = i2c_smbus_read_word_data(bat_client, 0x4d);
		printk(KERN_INFO "%s -- Hibernate V (0x4d) = %x\n",__func__ ,ret);
		msleep(50);

		/* set bq27541 to SEALED mode */
		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		printk(KERN_INFO "%s -- Change to Sealed Mode = %d\n",__func__ ,ret);
		msleep(50);
	}
}

static void bq27541_charger_reset(void)
{
	gpio_direction_output(BATT_LEARN, 1);
	msleep(100);
	gpio_direction_output(BATT_LEARN, 0);
	printk("Charger Reset with BATT_LEARN=%d\n", gpio_get_value(BATT_LEARN));
}

int bq27541_battery_check(int arg)
{
	int ret;
	int rsoc = 0;

	struct bq27541_device_info *di = i2c_get_clientdata(bat_client);
	if (!bat_client)
		dev_err(di->dev, "error getting bat_client\n");

	ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
	msleep(20);

	switch (arg) {
	case 1:
		return rsoc;
		break;
	case 2:
		/* Change to FULL at  97% due to capacity mapping. */
		if(rsoc < 97)
		{
			if(gpio_get_value(adapter))
				return POWER_SUPPLY_STATUS_CHARGING;
			else
				return POWER_SUPPLY_STATUS_DISCHARGING;
		}
		else
		{
			if(gpio_get_value(adapter))
				return POWER_SUPPLY_STATUS_FULL;
			else
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	default:
		return -EINVAL;
	}

}
EXPORT_SYMBOL(bq27541_battery_check);

int bq27541_low_temp_check(void)
{
	int ret, rsoc, temp;
	struct bq27541_device_info *di = i2c_get_clientdata(bat_client);

	if (!bat_client)
		dev_err(di->dev, "error getting bat_client\n");

	ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
	msleep(20);
	ret = bat_i2c_read(BQ27541_REG_TEMP, &temp, 0, di);
	msleep(20);

	if((rsoc > 40) && (temp < 2744) && (temp > 2647))
		return RSOC_NOT_QUALIFY;
	else if((rsoc <= 40) && (temp <= 2744) && (temp > 2647))
		return TEMP_UNDER_ZERO;
	else if((rsoc > 60) && (temp < 2647))
		return RSOC_NOT_QUALIFY;
	else if((rsoc <= 60) && (temp <= 2647))
		return TEMP_UNDER_NAT_TEN;
	else
		return REPORT_NORMAL_VAL;
}
EXPORT_SYMBOL(bq27541_low_temp_check);

static int bq27541_battery_health(struct bq27541_device_info *di)
{
	int ret;
	int status = 0;

	ret = bat_i2c_read(BQ27541_REG_FLAGS, &status, 0, di);
	msleep(20);

	if (ret  < 0) {
		dev_err(di->dev, "read failure\n");
		return ret;
	}

	if (ret & BQ27541_FLAG_SOCF)
		status = POWER_SUPPLY_HEALTH_DEAD;
	else if (ret & BQ27541_FLAG_OTC)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;

}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;
	int report = temp - 2731;

	ret = bat_i2c_read(BQ27541_REG_TEMP, &temp, 0, di);
	msleep(20);

	report = temp - 2731;
	bat_temp = report;
	if (ret < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	if((report == 0) || (report > 680)){
		dev_err(di->dev, "1st error temperature value\n");
		ret = bat_i2c_read(BQ27541_REG_TEMP, &temp, 0, di);
		msleep(50);
		if((report == 0) || (report > 680)){
			dev_err(di->dev, "2nd error temperature value\n");
			bat_temp = 273;
		}
		else
			bat_temp = temp - 2731;
	}
	else
		bat_temp = temp - 2731;

	return bat_temp;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bat_i2c_read(BQ27541_REG_VOLT, &volt, 0, di);
	msleep(20);

	if (ret < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27541_battery_current(struct bq27541_device_info *di)
{
	int ret;
	int curr = 0;

	ret = bat_i2c_read(BQ27541_REG_AI, &curr, 0, di);
	msleep(20);

	if (ret < 0) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}
	curr = (int)(s16)curr;

	return curr * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27541_battery_rsoc(struct bq27541_device_info *di)
{
	int ret, LTcheck;
	int rsoc = 0;

	ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
	msleep(20);

	if (ret < 0) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	Capacity = rsoc;
	if((rsoc == 0) || (rsoc > 100)){
		dev_err(di->dev, "1st error capacity value\n");
		ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
		msleep(50);
		if((rsoc == 0) || (rsoc > 100)){
			dev_err(di->dev, "2nd error capacity value\n");
			Capacity = old_rsoc;
		}
		else
			Capacity = rsoc;
	}
	else
		Capacity = rsoc;

	LTcheck = bq27541_low_temp_check();
	if((LTcheck == TEMP_UNDER_ZERO) || (LTcheck == TEMP_UNDER_NAT_TEN))
		return 0;
	else
	{
		/* Capacity mapping for 5% preserved engrgy */
		if (rsoc <= 23 && rsoc > 15)
			return (Capacity -= 2);
		else if (rsoc <= 15 && rsoc > 11)
			return (Capacity -= 3);
		else if (rsoc <= 11 && rsoc > 4)
			return (Capacity -= 4);
		else if (rsoc <= 4){
			Capacity = 0;
			return 0;
		}
		/* Capacity mapping from 97% to 100% */
		else if (rsoc >= 97){
			Capacity = 100;
			return Capacity;
		}
		else if (rsoc < 97 && rsoc >= 92)
			return (Capacity += 3);
		else if (rsoc < 92 && rsoc >= 87)
			return (Capacity += 2);
		else if (rsoc < 87 && rsoc >= 82)
			return (Capacity += 1);
		else
			return Capacity;
	}
}

static int bq27541_battery_status(struct bq27541_device_info *di)
{
	int ret;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = bat_i2c_read(BQ27541_REG_FLAGS, &status, 0, di);
	msleep(20);

	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return 0;
	}

	ret = bat_i2c_read(BQ27541_REG_SOC, &Capacity, 0, di);
	msleep(20);

	/* Query to judge AC status, not report to uevent. */
	if (ret < 0) {
		dev_err(di->dev, "error reading capacity\n");
		return 0;
	}

	/*
	 * PicassoMF will stop charging when reach 85 degree
	 * to start throttling. Once CPU temperature drops to
	 * 75 degree, battery driver helps to recover chargerIC.
	 */
	cpu_temp = tegra3_cpu_temp_query();
	if((acer_board_type == BOARD_PICASSO_MF) && gpio_get_value(adapter)){
		if(throttle_start && (cpu_temp <= 75)){
			bq27541_charger_reset();
			throttle_start = false;
		}
	}

	/* Change to FULL at  97% due to capacity mapping. */
	if(Capacity < 97)
	{
		if(gpio_get_value(adapter)){
			status = POWER_SUPPLY_STATUS_CHARGING;
			if(acer_board_type != BOARD_PICASSO_E2)
				gpio_direction_output(CHARGING_FULL, 1);
		}
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else
	{
		if(gpio_get_value(adapter)){
			status = POWER_SUPPLY_STATUS_FULL;
			if(acer_board_type != BOARD_PICASSO_E2)
				gpio_direction_output(CHARGING_FULL, 1);
		}
		else
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return status;

}

static int bq27541_battery_present(struct bq27541_device_info *di)
{
	int ret;
	int present = 0;

	/* Calculate counter for limitation of 10.5hr charger reset */
	if(acer_board_type != BOARD_PICASSO_E2){
		if(counter <= 240){
			if(gpio_get_value(adapter))
				counter += 1;
			else
				counter = 0;
		}
		else{
			bq27541_charger_reset();
			counter = 0;
		}
	}

	ret = bat_i2c_read(BQ27541_DESIGN_CAPACITY, &present, 0, di);
	msleep(20);
	design_capacity = present;

	if (ret >= 0){
		return 1;
	}
	else {
		dev_err(di->dev, "No Battery due to error reading design capacity! \n");
		return 0;
	}
}


#define bat_to_bq27541_device_info(x) container_of((x), \
				struct bq27541_device_info, bat);

static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = bat_to_bq27541_device_info(psy);

	mutex_lock(&di->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq27541_battery_status(di);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27541_battery_present(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq27541_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27541_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27541_battery_rsoc(di);
		old_rsoc = val->intval;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27541_battery_temperature(di);
		printk("AC = %d, Level = %d, RSOC = %d, Bat_temp = %d, CPU_temp = %d, LT = %d, Count = %d\n",
			gpio_get_value(adapter), old_rsoc, bq27541_battery_check(1), val->intval/10, cpu_temp,
			bq27541_low_temp_check(), counter);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq27541_battery_health(di);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = design_capacity;
		break;
	default:
		mutex_unlock(&di->lock);
		return -EINVAL;
	}
	mutex_unlock(&di->lock);

	return ret;
}

#define ac_to_bq27541_device_info(x) container_of((x), \
				struct bq27541_device_info, ac);

static enum power_supply_property bq27541_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *ac_power_supplied_to[] = {
	"bq27541-bat",
};

static int bq27541_get_ac_status(void)
{
	if(gpio_get_value(adapter))
		return ADAPTER_PLUG_IN;
	else
		return ADAPTER_PULL_OUT;
}

static int bq27541_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct bq27541_device_info *di = ac_to_bq27541_device_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq27541_get_ac_status();
		break;
	default:
		dev_err(&di->client->dev, "%s: INVALID property\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void bq27541_powersupply_init(struct bq27541_device_info *di)
{
	di->bat.name = "bq27541-bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27541_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27541_battery_props);
	di->bat.get_property = bq27541_battery_get_property;
	di->bat.external_power_changed = NULL;

	di->ac.name = "bq27541-ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.supplied_to = ac_power_supplied_to;
	di->ac.num_supplicants = ARRAY_SIZE(ac_power_supplied_to);
	di->ac.properties = bq27541_ac_props;
	di->ac.num_properties = ARRAY_SIZE(bq27541_ac_props);
	di->ac.get_property = bq27541_ac_get_property;
}

/*
 * i2c specific code
 */
static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[2];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;

	data[0] = reg;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].buf = data;

		if (!b_single)
			msg[1].len = 2;
		else
			msg[1].len = 1;

		msg[1].flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 2);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}

	return err;
}


static irqreturn_t ac_present_irq(int irq, void *data)
{
	struct bq27541_device_info *di = data;
	wake_lock_timeout(&ac_wake_lock, 3*HZ);
	/* Debounce with 80ms to block abnormal interrupt when AC plug out */
	schedule_delayed_work(&di->work, msecs_to_jiffies(DEBOUNCE));
	return IRQ_HANDLED;
}

static void debounce_work(struct work_struct *work)
{
	struct bq27541_device_info *di;
	di = container_of(work, struct bq27541_device_info, work.work);
	power_supply_changed(&di->ac);
}

static void bq27541_poll_timer_func(unsigned long pdi)
{
	struct bq27541_device_info *di = (void *)pdi;
	power_supply_changed(&di->bat);
	mod_timer(&di->battery_poll_timer, jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));
}

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27541_device_info *di;
	struct bq27541_access_methods *bus;
	int retval = 0;
	int err;

	printk(KERN_INFO "%s ++ \n",__func__);
	bat_client = client;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method data\n");
		retval = -ENOMEM;
		goto failed_device;
	}

	adapter = irq_to_gpio(client->irq);
	AC_IN = gpio_get_value(adapter);
	di->irq = client->irq;

	mutex_init(&di->lock);

	if(acer_board_type != BOARD_PICASSO_E2){
		tegra_gpio_enable(CHARGING_FULL);
		retval = gpio_request(CHARGING_FULL, "full_half_chg");
		if (retval < 0)
			printk(KERN_INFO "%s: gpio_request failed for gpio-%d\n",__func__, TEGRA_GPIO_PW5);
		mdelay(10);

		tegra_gpio_enable(CHARGER_STAT);
		retval = gpio_request(CHARGER_STAT, "chg_stat");
		if (retval < 0)
			printk(KERN_INFO "%s: gpio_request failed for gpio-%d\n",__func__, TEGRA_GPIO_PJ2);
		mdelay(10);
	}

	tegra_gpio_enable(BATT_LEARN);
	retval = gpio_request(BATT_LEARN, "batt_learn");
	if (retval < 0)
		printk(KERN_INFO "%s: gpio_request failed for gpio-%d\n",__func__, TEGRA_GPIO_PX6);

	if (client->dev.platform_data) {
		di->plat_data = kzalloc(sizeof(struct bq27541_platform_data), GFP_KERNEL);
		if (!di->plat_data) {
			dev_err(&client->dev, "failed to allocate platform data\n");
			retval = -ENOMEM;
			goto failed_bus;
		}
		memcpy(di->plat_data, client->dev.platform_data, sizeof(struct bq27541_platform_data));
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	di->bus = bus;
	di->client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "insufficient functionality!\n");
		retval = -ENODEV;
		goto failed_pdata;
	}

	bq27541_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto failed_pdata;
	}

	setup_timer(&di->battery_poll_timer, bq27541_poll_timer_func, (unsigned long) di);
	mod_timer(&di->battery_poll_timer, jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));

	retval = power_supply_register(&client->dev, &di->ac);
	if (retval) {
		dev_err(&client->dev, "failed to register ac power supply\n");
		goto failed_reg_bat;
	}

	INIT_DELAYED_WORK(&di->work, debounce_work);
	retval = request_threaded_irq(di->irq, NULL, ac_present_irq,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "ac_present", di);

	if (retval < 0) {
		dev_err(&di->client->dev, "%s: request_irq failed(%d)\n", __func__, retval);
		goto failed_reg_ac;
	}

	bq27541_dbg_kobj = kobject_create_and_add("dev-info_battery", NULL);
	if (bq27541_dbg_kobj == NULL)
	{
		dev_err(&di->client->dev,"%s: subsystem_register failed\n", __FUNCTION__);
	}
	err = sysfs_create_group(bq27541_dbg_kobj, &dbg_attr_group);
	if(err)
	{
		dev_err(&di->client->dev,"%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);
	}

	/* Reset Charger IC due to charging full within 4.5hr limitation */
	if(acer_board_type != BOARD_PICASSO_E2){
		if (gpio_get_value(adapter))
			bq27541_charger_reset();
	}

	wake_lock_init(&ac_wake_lock, WAKE_LOCK_SUSPEND, "t30-ac");
	dev_info(&client->dev, "support ver. %s enabled (%s)\n", DRIVER_VERSION, RELEASED_DATE);

#if defined(CONFIG_ARCH_ACER_T30)
	if ( (acer_board_type == BOARD_PICASSO_2 || acer_board_type == BOARD_PICASSO_M )
		&& (acer_board_id == BOARD_EVT || acer_board_id == BOARD_DVT1 || acer_board_id == BOARD_DVT2)) {
		bq27541_disable_hibernate();
	}
#endif

	return 0;

failed_reg_ac:
	power_supply_unregister(&di->ac);
failed_reg_bat:
	power_supply_unregister(&di->bat);
	del_timer_sync(&di->battery_poll_timer);
failed_pdata:
	kfree(di->plat_data);
failed_bus:
	kfree(bus);
failed_device:
	kfree(di);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	free_irq(di->irq, di);
	cancel_delayed_work_sync(&di->work);
	if(acer_board_type != BOARD_PICASSO_E2){
		gpio_free(CHARGING_FULL);
		gpio_free(CHARGER_STAT);
	}
	gpio_free(BATT_LEARN);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->bat);
	del_timer_sync(&di->battery_poll_timer);

	kfree(di->plat_data);
	kfree(di->bus);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27541_battery_suspend(struct i2c_client *client,
	pm_message_t state)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	mutex_lock(&di->lock);
	if(gpio_get_value(adapter) && (acer_board_type != BOARD_PICASSO_E2))
		gpio_direction_output(CHARGING_FULL, 0);
	del_timer_sync(&di->battery_poll_timer);
	enable_irq_wake(client->irq);
	return 0;
}

static int bq27541_battery_resume(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	setup_timer(&di->battery_poll_timer, bq27541_poll_timer_func, (unsigned long) di);
	mod_timer(&di->battery_poll_timer, jiffies + msecs_to_jiffies(BATTERY_FAST_POLL_PERIOD));
	disable_irq_wake(client->irq);
	mutex_unlock(&di->lock);
	return 0;
}
#endif


static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
#if defined(CONFIG_PM)
	.suspend	= bq27541_battery_suspend,
	.resume		= bq27541_battery_resume,
#endif
	.id_table = bq27541_id,
	.driver = {
		.name = "bq27541-battery",
	},
};

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Fail to register bq27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("StanleyTW Chang <StanleyTW_Chang@acer.com.tw>");
MODULE_DESCRIPTION("bq27541 battery driver");
MODULE_LICENSE("GPL");

#endif /* CONFIG_BATTERY_BQ27541 */

