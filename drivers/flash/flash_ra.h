/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_RA_H_
#define ZEPHYR_DRIVERS_FLASH_RA_H_

#include <zephyr/drivers/flash.h>
#include <instances/r_flash_hp.h>
#include <api/r_flash_api.h>

#if defined(CONFIG_SOC_SERIES_RA8)
#include "flash_ra8.h"
#include <zephyr/drivers/flash/ra_flash_api_extensions.h>
#endif

#if DT_PROP(DT_INST(0, soc_nv_flash), write_block_size)
#define FLASH_RA_WRITE_BLOCK_SIZE DT_PROP(DT_INST(0, soc_nv_flash), write_block_size)
#else
#error Flash write block size not available
/* Flash Write block size is extracted from device tree */
/* as flash node property 'write-block-size' */
#endif

#if defined(CONFIG_DUAL_BANK_MODE)
#define SOC_NV_FLASH_SIZE 0x2F8000
#else
#define SOC_NV_FLASH_SIZE DT_REG_SIZE(DT_INST(0, soc_nv_flash))
#endif

#define VECTOR_NUMBER_FCU_FRDYI ((IRQn_Type) 0) /* FCU FRDYI (Flash ready interrupt) */
#define VECTOR_NUMBER_FCU_FIFERR ((IRQn_Type) 1) /* FCU FIFERR (Flash access error interrupt) */

struct flash_hp_ra_data {
	struct st_flash_hp_instance_ctrl flash_ctrl;
	struct st_flash_cfg fsp_config;
};

struct flash_hp_ra_config {
	struct flash_parameters flash_ra_parameters;
};

int flash_ra_block_write(const struct device *dev, unsigned int offset, const void *data,
			 unsigned int len);
int flash_ra_block_erase(const struct device *dev, unsigned int offset, unsigned int len);

#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_ra_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
			  size_t *layout_size);
#endif

#if defined(CONFIG_FLASH_RA_WRITE_PROTECT)
int flash_ra_block_protect_set(const struct device *dev,
			       const struct flash_ra_ex_write_protect_in *request);
int flash_ra_block_protect_get(const struct device *dev,
			       struct flash_ra_ex_write_protect_out *response);
int flash_ra_ex_op_write_protect(const struct device *dev, const uintptr_t in, void *out);
#endif

#endif /* ZEPHYR_DRIVERS_FLASH_RA_H_ */
