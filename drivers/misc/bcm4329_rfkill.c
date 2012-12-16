/*
 * drivers/misc/bcm4329_rfkill.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/err.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
#include <linux/gpio.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#endif

struct bcm4329_rfkill_data {
	int gpio_reset;
	int gpio_shutdown;
	int delay;
	struct clk *bt_32k_clk;
#if defined(CONFIG_MACH_PICASSO_E) || defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
	int gpio_wifi_reset;
	int gpio_bcm_vdd;
#endif
#if defined(CONFIG_ARCH_ACER_T30)
    int gpio_ext_wake;
#endif
};

static struct bcm4329_rfkill_data *bcm4329_rfkill;


static struct kobject *devInfoBT_kobj;
char vendor_id[20];

static ssize_t vendor_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s += sprintf(s,"%s", vendor_id);
	pr_info("#%s  The vendor string %s length is %d", __FUNCTION__, s, s-buf);
	return (s - buf);
}

static ssize_t vendor_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	char *s = buf;
	pr_info("#%s  The vendor string %s length is %d", __FUNCTION__, buf, n);
	if (n <= 20)
	{
		memset(vendor_id,'\0',sizeof(vendor_id));
		memcpy(vendor_id, s, n);
	}
	return n;
}

#define debug_attr(_name) \
        static struct kobj_attribute _name##_attr = { \
        .attr = { \
        .name = __stringify(_name), \
        .mode = 0644, \
        }, \
        .show = _name##_show, \
        .store = _name##_store, \
        }

debug_attr(vendor);

static struct attribute * bt_group[] = {
        &vendor_attr.attr,
        NULL,
};

static struct attribute_group attr_bt_group = {
        .attrs = bt_group,
};

static void create_bt_dev_sys(void)
{
        int error;

        devInfoBT_kobj = kobject_create_and_add("dev-info_bt", NULL);
        if (devInfoBT_kobj == NULL)
                pr_info("## %s kobject_create_and_add failed\n", __FUNCTION__);

        error = sysfs_create_group(devInfoBT_kobj, &attr_bt_group);
        if (error)
                pr_info("## %s sysfs_create_group failed\n", __FUNCTION__);
        return;
}

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)

#define UART3_RX_GPIO    TEGRA_GPIO_PW7
#define UART3_TX_GPIO    TEGRA_GPIO_PW6
#define UART3_CTS_GPIO   TEGRA_GPIO_PA1
#define UART3_RTS_GPIO   TEGRA_GPIO_PC0

static void tegra_uart_fun_off(void)
{
        int gpio_rx, gpio_tx, gpio_cts, gpio_rts;

        gpio_rx = gpio_tx = 0;
        gpio_cts = gpio_rts = 0;

        gpio_rx = UART3_RX_GPIO;
        gpio_tx = UART3_TX_GPIO;
        gpio_cts = UART3_CTS_GPIO;
        gpio_rts = UART3_RTS_GPIO;

        if(gpio_rx) {
            tegra_gpio_enable(gpio_rx);
            tegra_gpio_enable(gpio_tx);
            tegra_gpio_enable(gpio_cts);
            tegra_gpio_enable(gpio_rts);
            gpio_request(gpio_rx,"uart_rx_gpio");
            gpio_request(gpio_tx,"uart_tx_gpio");
            gpio_request(gpio_cts,"uart_cts_gpio");
            gpio_request(gpio_rts,"uart_rts_gpio");
            gpio_direction_output(gpio_rx, 0);
            gpio_direction_output(gpio_tx, 0);
            gpio_direction_output(gpio_cts, 0);
            gpio_direction_output(gpio_rts, 0);
        }

        pr_info("%s: \n", __func__);

        return;
}

static void tegra_uart_fun_on(void)
{
        int gpio_rx, gpio_tx, gpio_cts, gpio_rts;

        gpio_rx = gpio_tx = 0;
        gpio_cts = gpio_rts = 0;

        gpio_rx = UART3_RX_GPIO;
        gpio_tx = UART3_TX_GPIO;
        gpio_cts = UART3_CTS_GPIO;
        gpio_rts = UART3_RTS_GPIO;

        if(gpio_rx) {
            tegra_gpio_disable(gpio_rx);
            tegra_gpio_disable(gpio_tx);
            tegra_gpio_disable(gpio_cts);
            tegra_gpio_disable(gpio_rts);
            gpio_free(gpio_rx);
            gpio_free(gpio_tx);
            gpio_free(gpio_cts);
            gpio_free(gpio_rts);
        }

        pr_info("%s: \n", __func__);

        return;
}
#endif


#ifdef BT_DEBUG
static void bcm4330_gpio_state(void)
{
    pr_info("%s: bcm4329_rfkill->gpio_shutdown = %d.\n", __func__,gpio_get_value(bcm4329_rfkill->gpio_shutdown));
    pr_info("%s: bcm4329_rfkill->gpio_reset = %d.\n", __func__,gpio_get_value(bcm4329_rfkill->gpio_reset));
    pr_info("%s: bcm4329_rfkill->gpio_wifi_vdd = %d.\n", __func__,gpio_get_value(bcm4329_rfkill->gpio_wifi_vdd));
    pr_info("%s: bcm4329_rfkill->gpio_wifi_reset = %d.\n", __func__,gpio_get_value(bcm4329_rfkill->gpio_wifi_reset));
}
#endif

static int bcm4329_bt_rfkill_set_power(void *data, bool blocked)
{
	/*
	 * check if BT gpio_shutdown line status and current request are same.
	 * If same, then return, else perform requested operation.
	 */
	if (gpio_get_value(bcm4329_rfkill->gpio_shutdown) && !blocked)
		return 0;

	if (blocked) {
		pr_info("%s: BT Power off.\n", __func__);

#if defined(CONFIG_ARCH_ACER_T30)
		if (bcm4329_rfkill->gpio_ext_wake) {
			gpio_direction_output(bcm4329_rfkill->gpio_ext_wake, 0);
			pr_info("%s: bcm4329_rfkill->gpio_ext_wake = 0.\n", __func__);
		}
#endif

		if (bcm4329_rfkill->gpio_shutdown) {
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 0);
			pr_info("%s: bcm4329_rfkill->gpio_shutdown = 0.\n", __func__);
		}

		if (bcm4329_rfkill->gpio_reset) {
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 0);
			pr_info("%s: bcm4329_rfkill->gpio_reset = 0.\n", __func__);
		}

		if (bcm4329_rfkill->bt_32k_clk) {
			clk_disable(bcm4329_rfkill->bt_32k_clk);
			pr_info("%s: bcm4329_rfkill->bt_32k_clk = disable.\n", __func__);
        }

#if defined(CONFIG_MACH_PICASSO_E) || defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
		if (bcm4329_rfkill->gpio_bcm_vdd) {
			if (!gpio_get_value(bcm4329_rfkill->gpio_wifi_reset)) {
				gpio_direction_output(bcm4329_rfkill->gpio_bcm_vdd, 0);
				pr_info("%s: bcm4329_rfkill->gpio_bcm_vdd = 0.\n", __func__);
			}
		}
#endif

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
		tegra_uart_fun_off();
#endif

	    } else {
		pr_info("%s: BT Power on.\n", __func__);

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
		tegra_uart_fun_on();
#endif

#if defined(CONFIG_MACH_PICASSO_E) || defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
		if (bcm4329_rfkill->gpio_bcm_vdd) {
			if (!gpio_get_value(bcm4329_rfkill->gpio_wifi_reset)) {
				pr_info("%s: bcm4329_rfkill->gpio_bcm_vdd = 1.\n", __func__);
				gpio_direction_output(bcm4329_rfkill->gpio_bcm_vdd, 1);
				mdelay(50);
			}
		}
#endif
        if (bcm4329_rfkill->bt_32k_clk) {
			clk_enable(bcm4329_rfkill->bt_32k_clk);
			pr_info("%s: bcm4329_rfkill->bt_32k_clk = enable.\n", __func__);
        }

		if (bcm4329_rfkill->gpio_shutdown)
		{
			pr_info("%s: bcm4329_rfkill->gpio_shutdown = 1.\n", __func__);
			gpio_direction_output(bcm4329_rfkill->gpio_shutdown, 1);
#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
			msleep(100);
#endif
		}
		if (bcm4329_rfkill->gpio_reset)
		{
			pr_info("%s: bcm4329_rfkill->gpio_reset = 1.\n", __func__);
			gpio_direction_output(bcm4329_rfkill->gpio_reset, 1);
		}
	}
	return 0;
}

static const struct rfkill_ops bcm4329_bt_rfkill_ops = {
	.set_block = bcm4329_bt_rfkill_set_power,
};

static int bcm4329_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *bt_rfkill;
	struct resource *res;
	int ret;
	bool enable = false;  /* off */
	bool default_sw_block_state;
	create_bt_dev_sys();

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
	tegra_uart_fun_off();
#endif

	bcm4329_rfkill = kzalloc(sizeof(*bcm4329_rfkill), GFP_KERNEL);
	if (!bcm4329_rfkill)
		return -ENOMEM;

	bcm4329_rfkill->bt_32k_clk = clk_get(&pdev->dev, "bcm4329_32k_clk");
	if (IS_ERR(bcm4329_rfkill->bt_32k_clk)) {
		pr_warn("%s: can't find bcm4329_32k_clk.\
				assuming 32k clock to chip\n", __func__);
		bcm4329_rfkill->bt_32k_clk = NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_nreset_gpio");
	if (res) {
		bcm4329_rfkill->gpio_reset = res->start;
		tegra_gpio_enable(bcm4329_rfkill->gpio_reset);
		ret = gpio_request(bcm4329_rfkill->gpio_reset,
						"bcm4329_nreset_gpio");
	} else {
		pr_warn("%s : can't find reset gpio. "
			"reset gpio may not be defined for "
			"this platform \n", __func__);
		bcm4329_rfkill->gpio_reset = 0;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
						"bcm4329_nshutdown_gpio");
	if (res) {
		bcm4329_rfkill->gpio_shutdown = res->start;
		tegra_gpio_enable(bcm4329_rfkill->gpio_shutdown);
		ret = gpio_request(bcm4329_rfkill->gpio_shutdown,
						"bcm4329_nshutdown_gpio");
	} else {
		pr_warn("%s : can't find shutdown gpio "
			"shutdown gpio may not be defined for "
			"this platform \n", __func__);
		bcm4329_rfkill->gpio_shutdown = 0;
	}

#if defined(CONFIG_MACH_PICASSO_E) || defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M) || defined(CONFIG_MACH_PICASSO_E2) || defined(CONFIG_MACH_PICASSO_MF)
	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
					"bcm4329_wifi_reset_gpio");
	if (res) {
		bcm4329_rfkill->gpio_wifi_reset = res->start;
	} else {
		pr_warn("%s : can't find wifi_reset gpio.\n", __func__);
		bcm4329_rfkill->gpio_wifi_reset = 0;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
			"bcm4329_vdd_gpio");
	if (res) {
		bcm4329_rfkill->gpio_bcm_vdd = res->start;
		gpio_set_value(bcm4329_rfkill->gpio_bcm_vdd, 0);
	} else {
		pr_warn("%s : can't find bcm_vdd gpio.\n", __func__);
		bcm4329_rfkill->gpio_bcm_vdd = 0;
	}
#endif

#if defined(CONFIG_ARCH_ACER_T30)
	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
			"bcm4329_ext_wake_gpio");
	if (res) {
		bcm4329_rfkill->gpio_ext_wake = res->start;
		gpio_set_value(bcm4329_rfkill->gpio_ext_wake, 0);
	} else {
		pr_warn("%s : can't find ext_wake gpio.\n", __func__);
		bcm4329_rfkill->gpio_ext_wake = 0;
	}
#endif
	/* make sure at-least one of the GPIO is defined */
	if (!bcm4329_rfkill->gpio_reset && !bcm4329_rfkill->gpio_shutdown)
		goto free_bcm_res;

	if (bcm4329_rfkill->bt_32k_clk && enable)
		clk_enable(bcm4329_rfkill->bt_32k_clk);
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_direction_output(bcm4329_rfkill->gpio_shutdown, enable);
	if (bcm4329_rfkill->gpio_reset)
		gpio_direction_output(bcm4329_rfkill->gpio_reset, enable);

	bt_rfkill = rfkill_alloc("bcm4329 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm4329_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill))
		goto free_bcm_res;

	default_sw_block_state = !enable;
	rfkill_set_states(bt_rfkill, default_sw_block_state, false);

	ret = rfkill_register(bt_rfkill);

	if (unlikely(ret)) {
		rfkill_destroy(bt_rfkill);
		goto free_bcm_res;
	}

	return 0;

free_bcm_res:
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_free(bcm4329_rfkill->gpio_shutdown);
	if (bcm4329_rfkill->gpio_reset)
		gpio_free(bcm4329_rfkill->gpio_reset);
	if (bcm4329_rfkill->bt_32k_clk && enable)
		clk_disable(bcm4329_rfkill->bt_32k_clk);
	if (bcm4329_rfkill->bt_32k_clk)
		clk_put(bcm4329_rfkill->bt_32k_clk);
	kfree(bcm4329_rfkill);
	return -ENODEV;
}

static int bcm4329_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *bt_rfkill = platform_get_drvdata(pdev);

	if (bcm4329_rfkill->bt_32k_clk)
		clk_put(bcm4329_rfkill->bt_32k_clk);
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);
	if (bcm4329_rfkill->gpio_shutdown)
		gpio_free(bcm4329_rfkill->gpio_shutdown);
	if (bcm4329_rfkill->gpio_reset)
		gpio_free(bcm4329_rfkill->gpio_reset);
	kfree(bcm4329_rfkill);

	return 0;
}

static struct platform_driver bcm4329_rfkill_driver = {
	.probe = bcm4329_rfkill_probe,
	.remove = bcm4329_rfkill_remove,
	.driver = {
		   .name = "bcm4329_rfkill",
		   .owner = THIS_MODULE,
	},
};

static int __init bcm4329_rfkill_init(void)
{
	return platform_driver_register(&bcm4329_rfkill_driver);
}

static void __exit bcm4329_rfkill_exit(void)
{
	platform_driver_unregister(&bcm4329_rfkill_driver);
}

module_init(bcm4329_rfkill_init);
module_exit(bcm4329_rfkill_exit);

MODULE_DESCRIPTION("BCM4329 rfkill");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");
