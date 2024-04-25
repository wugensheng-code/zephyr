/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_RA8_H_
#define ZEPHYR_DRIVERS_FLASH_RA8_H_

#include <zephyr/drivers/flash.h>
#include <instances/r_flash_hp.h>
#include <api/r_flash_api.h>

#define FLASH_HP_FENTRYR_PE_MODE_BITS  (0x0081U)

#define FLASH_HP_BANK2_OFFSET                                                                      \
	(BSP_FEATURE_FLASH_HP_CF_DUAL_BANK_START - BSP_FEATURE_FLASH_CODE_FLASH_START)

#define FLASH_HP_CF_BLOCK_8KB_SIZE  BSP_FEATURE_FLASH_HP_CF_REGION0_BLOCK_SIZE
#define FLASH_HP_CF_BLOCK_32KB_SIZE BSP_FEATURE_FLASH_HP_CF_REGION1_BLOCK_SIZE

#define FLASH_HP_CF_BLOCK_8KB_LOW_START (0)
#define FLASH_HP_CF_BLOCK_8KB_LOW_END   (7)
#define FLASH_HP_CF_BLOCK_8KB_HIGH_START (70)
#define FLASH_HP_CF_BLOCK_8KB_HIGH_END   (77)

#define FLASH_HP_FCU_CONFIG_SET_BPS     (0x1300A1C0U)
#define FLASH_HP_FCU_CONFIG_SET_BPS_SEC (0x0300A240U)
#define FLASH_HP_FCU_CONFIG_SET_BPS_SEL (0x0300A2C0U)

#define FLASH_HP_FCU_CONFIG_SET_PBPS     (0x1300A1E0U)
#define FLASH_HP_FCU_CONFIG_SET_PBPS_SEC (0x0300A260U)

/* Zero based offset into g_configuration_area_data[] for BPS */
#define FLASH_HP_FCU_CONFIG_SET_BPS_OFFSET (0U)

#if (BSP_ROM_SIZE_BYTES == 2016 * 1024)
#define FLASH_HP_CF_BLOCK_32KB_LINEAR_START    (8)
#define FLASH_HP_CF_BLOCK_32KB_LINEAR_END      (68)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_LOW_START  (8)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_LOW_END    (36)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_HIGH_START (78)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_HIGH_END   (106)
#elif (BSP_ROM_SIZE_BYTES == 1024 * 1024)
#define FLASH_HP_CF_BLOCK_32KB_LINEAR_START    (8)
#define FLASH_HP_CF_BLOCK_32KB_LINEAR_END      (37)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_LOW_START  (8)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_LOW_END    (21)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_HIGH_START (78)
#define FLASH_HP_CF_BLOCK_32KB_DUAL_HIGH_END   (91)
#else
#error "Please check 'Flash Size in KB' under 'General Architecture Options'"
#endif

fsp_err_t R_FLASH_HP_BlockProtectSet(flash_ctrl_t *const p_api_ctrl, uint8_t *bps_val_ns,
				     uint8_t *bps_val_sec, uint8_t *bps_val_sel,
				     uint8_t *pbps_val_ns, uint8_t *pbps_val_sec, uint32_t size);
fsp_err_t R_FLASH_HP_BlockProtectGet(flash_ctrl_t *const p_api_ctrl, uint32_t *bps_val_ns,
				     uint32_t *bps_val_sec, uint8_t *bps_val_sel,
				     uint8_t *pbps_val_ns, uint8_t *pbps_val_sec, uint32_t *size);

#endif /* ZEPHYR_DRIVERS_FLASH_RA8_H_ */
