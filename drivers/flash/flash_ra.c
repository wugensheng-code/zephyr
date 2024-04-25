/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash/ra_flash_api_extensions.h>
#include <zephyr/irq.h>

#include "flash_ra.h"

#if defined(CONFIG_SOC_SERIES_RA8)
#define DT_DRV_COMPAT renesas_ra8_flash_controller
#endif

LOG_MODULE_REGISTER(flash_ra, CONFIG_FLASH_LOG_LEVEL);

static int flash_ra_init(const struct device *dev);
static int flash_ra_erase(const struct device *dev, off_t offset, size_t len);
static int flash_ra_read(const struct device *dev, off_t offset, void *data, size_t len);
static int flash_ra_write(const struct device *dev, off_t offset, const void *data, size_t len);
static const struct flash_parameters *flash_ra_get_parameters(const struct device *dev);
void bgo_callback(flash_callback_args_t *p_args);

#ifdef CONFIG_FLASH_EX_OP_ENABLED
static int flash_ra_ex_op(const struct device *dev, uint16_t code, const uintptr_t in, void *out);
#endif

static struct flash_hp_ra_data flash_hp_ra_data = {.fsp_config = {
								.data_flash_bgo = true,
								.p_callback = bgo_callback,
								.p_context = NULL,
#if defined(VECTOR_NUMBER_FCU_FRDYI)
								.irq = VECTOR_NUMBER_FCU_FRDYI,
#else
								.irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_FCU_FIFERR)
								.err_irq = VECTOR_NUMBER_FCU_FIFERR,
#else
								.err_irq = FSP_INVALID_VECTOR,
#endif
								.err_ipl = (3),
								.ipl = (3),
						   }};

static struct flash_hp_ra_config flash_hp_ra_config = {
	.flash_ra_parameters = {
		.write_block_size = FLASH_RA_WRITE_BLOCK_SIZE,
		.erase_value = 0xff,
	}
};

static const struct flash_driver_api flash_ra_api = {
	.erase = flash_ra_erase,
	.write = flash_ra_write,
	.read = flash_ra_read,
	.get_parameters = flash_ra_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_ra_page_layout,
#endif
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_ra_ex_op,
#endif
};

static int flash_ra_init(const struct device *dev)
{
	fsp_err_t err = FSP_SUCCESS;
	struct flash_hp_ra_data *data = dev->data;

	err = R_FLASH_HP_Open(&data->flash_ctrl, &data->fsp_config);
	LOG_DBG("flash: open error=%d", err);

	if (err != FSP_SUCCESS) {
		return -ENODEV;
	}

	return 0;
}

static int flash_ra_erase(const struct device *dev, off_t offset, size_t len)
{
	int err;

	if ((offset < 0) || offset >= SOC_NV_FLASH_SIZE || (SOC_NV_FLASH_SIZE - offset) < len) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("flash: erase 0x%lx, len: %u", (long)(offset + CONFIG_FLASH_BASE_ADDRESS), len);

	err = flash_ra_block_erase(dev, offset, len);

	return err;
}

static int flash_ra_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	if ((offset < 0) || offset >= SOC_NV_FLASH_SIZE || (SOC_NV_FLASH_SIZE - offset) < len) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("flash: read 0x%lx, len: %u", (long)(offset + CONFIG_FLASH_BASE_ADDRESS), len);

	memcpy(data, (uint8_t *)(offset + CONFIG_FLASH_BASE_ADDRESS), len);

	return 0;
}

static int flash_ra_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	int err;

	if ((offset < 0) || offset >= SOC_NV_FLASH_SIZE || (SOC_NV_FLASH_SIZE - offset) < len) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("flash: write 0x%lx, len: %u", (long)(offset + CONFIG_FLASH_BASE_ADDRESS), len);

	err = flash_ra_block_write(dev, (offset + CONFIG_FLASH_BASE_ADDRESS), data, len);

	return err;
}

static const struct flash_parameters *flash_ra_get_parameters(const struct device *dev)
{
	const struct flash_hp_ra_config *config = dev->config;

	return &config->flash_ra_parameters;
}

#ifdef CONFIG_FLASH_EX_OP_ENABLED
static int flash_ra_ex_op(const struct device *dev, uint16_t code, const uintptr_t in, void *out)
{
	int err = -ENOTSUP;

	switch (code) {
#if defined(CONFIG_FLASH_RA_WRITE_PROTECT)
	case FLASH_RA_EX_OP_WRITE_PROTECT:
		err = flash_ra_ex_op_write_protect(dev, in, out);
		break;
#endif /* CONFIG_FLASH_RA_WRITE_PROTECT */

	default:
		break;
	}

	return err;
}
#endif

void bgo_callback(flash_callback_args_t *p_args)
{
	/*Call back function for flash */
}

DEVICE_DT_INST_DEFINE(0, flash_ra_init, NULL, &flash_hp_ra_data, &flash_hp_ra_config, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_ra_api);
