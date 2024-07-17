/*
 * Copyright (c) 2022 STMicroelectronics
 * Copyright (c) 2022 Georgij Cernysiov
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_ospi_nor

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/dt-bindings/flash_controller/ospi.h>
#include <zephyr/irq.h>

#include <r_ospi_b.h>

#include "spi_nor.h"
#include "jesd216.h"


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(renesas_ra_ospi_nor, CONFIG_FLASH_LOG_LEVEL);

#define RA_OSPI_NODE DT_INST_PARENT(0)

struct flash_ra_ospi_config {
	size_t flash_size;
	spi_flash_protocol_t protocol;
	int data_rate; /* DTR or STR */
	const struct pinctrl_dev_config *pcfg;
#if DT_NODE_HAS_PROP(DT_INST(0, renesas_ra_ospi_nor), sfdp_bfp)
	uint8_t sfdp_bfp[DT_INST_PROP_LEN(0, sfdp_bfp)];
#endif /* sfdp_bfp */
};

struct flash_ra_ospi_data {
	R_XSPI_Type *ospi;
	ospi_b_instance_ctrl_t fsp_cfg;
	spi_flash_cfg_t flash_cfg;
	ospi_b_extended_cfg_t ext_cfg;
	ospi_b_timing_setting_t timint_settings;
	struct k_sem sem;
	struct k_sem sync;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
	struct jesd216_erase_type erase_types[JESD216_NUM_ERASE_TYPES];
	/* Number of bytes per page */
	uint16_t page_size;
	/* Address width in bytes */
	uint8_t address_width;
	/* Read operation dummy cycles */
	uint8_t read_dummy;
	uint32_t read_opcode;
	uint32_t write_opcode;
	enum jesd216_mode_type read_mode;
	enum jesd216_dw15_qer_type qer_type;
#if defined(CONFIG_FLASH_JESD216_API)
	/* Table to hold the jedec Read ID given by the octoFlash or the DTS */
	uint8_t jedec_id[JESD216_READ_ID_LEN];
#endif /* CONFIG_FLASH_JESD216_API */
	int cmd_status;
};

static int ra_ospi_read_sfdp(const struct device *dev, off_t addr,
		void *data, size_t size)
{
	// struct flash_ra_ospi_data *dev_data = dev->data;
	// fsp_err_t fsp_err;
	// spi_flash_direct_transfer_t const transfer = {
    // 	.command        = JESD216_CMD_READ_SFDP,
    // 	.address        = addr,
    // 	.data           = 0,
    // 	.command_length = 1U,
    // 	.address_length = 4U,
    // 	.data_length    = 0U,
    // 	.dummy_cycles   = 0U
	// };

	// fsp_err = R_OSPI_B_DirectTransfer(&dev_data->fsp_cfg, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	// if (fsp_err != FSP_SUCCESS) {
	// 	LOG_ERR("%d: Failed to send OSPI instruction", fsp_err);
	// }

	// transfer.command = 0U,
	// transfer.command_length = 0U,
	// transfer.address = 0U;
	// transfer.address_length = 0U;
	// transfer.data_length = 4U;

	// fsp_err = R_OSPI_B_DirectTransfer(&dev_data->fsp_cfg, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	// if (fsp_err != FSP_SUCCESS) {
	// 	LOG_ERR("%d: Failed to send OSPI instruction", fsp_err);
	// }

	// data = &transfer.data;

	return 0;
}

static int ospi_read_sfdp(const struct device *dev, off_t addr, void *data,
			  size_t size)
{
#if DT_NODE_HAS_PROP(DT_INST(0, renesas_ra_ospi_nor), sfdp_bfp)

#else
	LOG_INF("Read SFDP from Flash");
	/* Get the SFDP from the octoFlash (no sfdp-bfp table in the DeviceTree) */
	if (ra_ospi_read_sfdp(dev, addr, data, size) == 0) {
		/* If valid, then ignore any table from the DTS */
		return 0;
	}
	LOG_INF("Error reading SFDP from Flash and none in the DTS");
	return -EINVAL;
#endif
}


static int flash_ra_ospi_init(const struct device *dev)
{
	const struct flash_ra_ospi_config *cfg = dev->config;
	struct flash_ra_ospi_data *data = dev->data;
	int ret;
	fsp_err_t fsp_err;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("OSPI pinctrl setup failed (%d)", ret);
		return ret;
	}


	fsp_err = R_OSPI_B_Open(&data->fsp_cfg, &data->flash_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("OSPI Open failed (%d)", fsp_err);
	}

	fsp_err = R_OSPI_B_SpiProtocolSet(&data->fsp_cfg, SPI_FLASH_PROTOCOL_EXTENDED_SPI);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("OSPI Set protocol Extended SPI mode (commands on 1 line) failed (%d)", fsp_err);
	}

	spi_flash_direct_transfer_t const transfer = {
    	.command        = SPI_NOR_CMD_RDID,
    	.address        = 0,
    	.data           = 0,
    	.command_length = 1,
    	.address_length = 0,
    	.data_length    = 4,
    	.dummy_cycles   = 0
	};

	fsp_err = R_OSPI_B_DirectTransfer(&data->fsp_cfg, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("failed to read Flash id (%d)", fsp_err);
	}

	LOG_INF("FLASH ID:%.2x, len:%d", transfer.data, transfer.data_length);



	/* Send the instruction to read the SFDP  */
	const uint8_t decl_nph = 2;
	union {
		/* We only process BFP so use one parameter block */
		uint8_t raw[JESD216_SFDP_SIZE(decl_nph)];
		struct jesd216_sfdp_header sfdp;
	} u;
	const struct jesd216_sfdp_header *hp = &u.sfdp;

	ret = ospi_read_sfdp(dev, 0, u.raw, sizeof(u.raw));
	if (ret != 0) {
		LOG_ERR("SFDP read failed: %d", ret);
		return ret;
	}

	uint32_t magic = jesd216_sfdp_magic(hp);

	if (magic != JESD216_SFDP_MAGIC) {
		LOG_ERR("SFDP magic %08x invalid", magic);
		return -EINVAL;
	}

	LOG_DBG("%s: SFDP v %u.%u AP %x with %u PH", dev->name,
		hp->rev_major, hp->rev_minor, hp->access, 1 + hp->nph);

	return 0;
}

PINCTRL_DT_DEFINE(RA_OSPI_NODE);

static const struct flash_ra_ospi_config flash_ra_ospi_cfg = {
	.flash_size = DT_INST_REG_SIZE_BY_IDX(0, 0),
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(RA_OSPI_NODE),
};

static struct flash_ra_ospi_data flash_ra_ospi_data = {
	.ospi = (R_XSPI_Type *)DT_REG_ADDR(RA_OSPI_NODE),
	.flash_cfg.spi_protocol = SPI_FLASH_PROTOCOL_EXTENDED_SPI,
	.flash_cfg.read_mode = SPI_FLASH_READ_MODE_STANDARD,
	.flash_cfg.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,
	.flash_cfg.dummy_clocks = SPI_FLASH_DUMMY_CLOCKS_DEFAULT,
	.flash_cfg.p_extend = &flash_ra_ospi_data.ext_cfg,
	.ext_cfg.channel = OSPI_B_DEVICE_NUMBER_1,
	.ext_cfg.p_timing_settings = &flash_ra_ospi_data.timint_settings,
	.ext_cfg.read_dummy_cycles = 0,
	.ext_cfg.program_dummy_cycles = 0,
	.ext_cfg.status_dummy_cycles = 0,
	.timint_settings.cs_pullup_lag = OSPI_B_COMMAND_CS_PULLUP_CLOCKS_NO_EXTENSION,
	.timint_settings.cs_pulldown_lead = OSPI_B_COMMAND_CS_PULLDOWN_CLOCKS_NO_EXTENSION,
	.timint_settings.command_to_command_interval =  OSPI_B_COMMAND_INTERVAL_CLOCKS_2,
};

DEVICE_DT_INST_DEFINE(0, &flash_ra_ospi_init, NULL,
		      &flash_ra_ospi_data, &flash_ra_ospi_cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
