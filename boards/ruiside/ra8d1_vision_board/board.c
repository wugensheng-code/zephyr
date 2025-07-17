/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/platform/hooks.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#if CONFIG_BOARD_EARLY_INIT_HOOK
void board_early_init_hook(void)
{
    const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(ioporta));
	if (device_is_ready(dev)) {
		printk("GPIO device not ready\n");
		return;
	}

	gpio_pin_configure(dev, 13, GPIO_OUTPUT_ACTIVE);
	return;
}
#endif
