/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra8_sdhi

#include <zephyr/drivers/sdhc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <r_sdhi.h>
#include <r_dmac.h>
#include <r_transfer_api.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sdhi, CONFIG_SDHC_LOG_LEVEL);

#define SDHI_PRV_SD_INFO2_CBSY_SDD0MON_IDLE_MASK           (0x4080)
#define SDHI_PRV_SD_INFO2_CBSY_SDD0MON_IDLE_VAL            (0x80)

#define SDHI_PRV_SD_CLK_CTRL_DEFAULT                       (0x20U)

/* SD_INFO1 */
#define SDHI_PRV_SDHI_INFO1_RESPONSE_END                   (1U << 0)  // Response End
#define SDHI_PRV_SDHI_INFO1_ACCESS_END                     (1U << 2)  // Access End

#define SDHI_PRV_SDHI_INFO2_MASK_CMD_SEND                  (0x00007C80U)
#define SDHI_PRV_SDHI_PRV_SD_CLK_AUTO_CLOCK_ENABLE_MASK    (0x300U)
#define SDHI_PRV_RESPONSE_BIT                              (0U)
#define SDHI_PRV_SD_STOP_SD_SECCNT_ENABLE                  (0x100U)

struct sdhi_ra_config {
	const struct pinctrl_dev_config *pincfg;
	uint32_t max_frequency;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};

struct sdhi_ra_data {
	struct sdhc_host_props props;
	struct k_sem transfer_sem;
	struct k_mutex access_mutex;

	sdhi_instance_ctrl_t sdhi;
	sdmmc_cfg_t fsp_config;
	transfer_instance_t transfer;
	dmac_instance_ctrl_t transfer_ctrl;
	transfer_info_t transfer_info;
	transfer_cfg_t transfer_cfg;
	dmac_extended_cfg_t transfer_extend;
};

/* convert sd_voltage to string */
static inline const char *const sdhi_ra_get_signal_voltage_str(enum sd_voltage voltage)
{
	static const char *const sig_vol_str[] = {
		[0] = "Unset",		 [SD_VOL_3_3_V] = "3.3V", [SD_VOL_3_0_V] = "3.0V",
		[SD_VOL_1_8_V] = "1.8V", [SD_VOL_1_2_V] = "1.2V",
	};

	if (voltage >= 0 && voltage < ARRAY_SIZE(sig_vol_str)) {
		return sig_vol_str[voltage];
	} else {
		return "Unknown";
	}
}

/* convert sdhc_timing_mode to string */
static inline const char *const sdhi_ra_get_timing_str(enum sdhc_timing_mode timing)
{
	static const char *const timing_str[] = {
		[0] = "Unset",
		[SDHC_TIMING_LEGACY] = "LEGACY",
		[SDHC_TIMING_HS] = "HS",
		[SDHC_TIMING_SDR12] = "SDR12",
		[SDHC_TIMING_SDR25] = "SDR25",
		[SDHC_TIMING_SDR50] = "SDR50",
		[SDHC_TIMING_SDR104] = "SDR104",
		[SDHC_TIMING_DDR50] = "DDR50",
		[SDHC_TIMING_DDR52] = "DDR52",
		[SDHC_TIMING_HS200] = "HS200",
		[SDHC_TIMING_HS400] = "HS400",
	};

	if (timing >= 0 && timing < ARRAY_SIZE(timing_str)) {
		return timing_str[timing];
	} else {
		return "Unknown";
	}
}


static int sdhi_ra_isr(const struct device *dev)
{
	return 0;
}

static int sdhi_ra_reset(const struct device *dev)
{
	const struct sdhi_ra_data *data = dev->data;

	k_mutex_lock(&data->access_mutex, K_FOREVER);

	data->sdhi.p_reg->SOFT_RST = 0x0U;
	data->sdhi.p_reg->SOFT_RST = 0x1U;

	k_mutex_unlock(&data->access_mutex);

	return 0;
}

static int sdhi_ra_get_host_props(const struct device *dev,
	struct sdhc_host_props *props)
{
	struct sdhi_ra_data *data;

	if (!props || !dev || !dev->data) {
		return -EINVAL;
	}

	data = dev->data;
	memcpy(props, &data->props, sizeof(*props));
	return 0;
}

static int clock_rate_set(sdhi_instance_ctrl_t *p_ctrl, uint32_t max_rate)
{
    uint32_t setting = 0xFFU;

    /* Get the runtime frequency of the source of the SD clock */
    uint32_t frequency = R_FSP_SystemClockHzGet(BSP_FEATURE_SDHI_CLOCK);

    /* Iterate over all possible divisors, starting with the smallest, until the resulting clock rate is less than
     * or equal to the requested maximum rate. */
    for (uint32_t divisor_shift = BSP_FEATURE_SDHI_MIN_CLOCK_DIVISION_SHIFT;
            divisor_shift <= 9U;
            divisor_shift++)
    {
        if ((frequency >> divisor_shift) <= max_rate)
        {
            /* If the calculated frequency is less than or equal to the maximum supported by the device,
             * select this frequency. The register setting is the divisor value divided by 4, or 0xFF for no divider. */
            setting = divisor_shift ? ((1U << divisor_shift) >> 2U) : UINT8_MAX;

            /* Set the clock setting. */

            /* The clock register is accessible 8 SD clock counts after the last command completes.  Each register access
             * requires at least one PCLK count, so check the register up to 8 times the maximum PCLK divisor value (512). */
            uint32_t timeout = 8U * 512U;

            while (timeout > 0U)
            {
                /* Do not write to clock control register until this bit is set. */
                if (p_ctrl->p_reg->SD_INFO2_b.SD_CLK_CTRLEN)
                {
                    /* Set the calculated divider and enable clock output to start the 74 clocks required before
                     * initialization. Do not change the automatic clock control setting. */
                    uint32_t clkctrlen = p_ctrl->p_reg->SD_CLK_CTRL & (1U << 9);
                    p_ctrl->p_reg->SD_CLK_CTRL = setting | clkctrlen | (1U << 8);
                    p_ctrl->device.clock_rate = frequency >> divisor_shift;

                    return 0;
                }

                timeout--;
            }

            /* Valid setting already found, stop looking. */
            break;
        }
    }

    return -EIO;
}

static int sdhi_ra_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhi_ra_data *data = dev->data;

	if (!dev || !ios || !dev->data || !dev->config) {
		return -EINVAL;
	}

	LOG_DBG("SDHC I/O: bus width %d, clock %dHz, card power %s, "
		"timing %s, voltage %s",
		ios->bus_width, ios->clock, ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		sdhi_ra_get_timing_str(ios->timing),
		sdhi_ra_get_signal_voltage_str(ios->signal_voltage));

	/* reset sdhi */
	data->sdhi.p_reg->SOFT_RST = 0x0U;
	data->sdhi.p_reg->SOFT_RST = 0x01U;
	data->sdhi.p_reg->SD_CLK_CTRL = SDHI_PRV_SD_CLK_CTRL_DEFAULT;
	data->sdhi.p_reg->SDIO_MODE = 0x00U;
	data->sdhi.p_reg->SD_DMAEN = 0x00U;
	data->sdhi.p_reg->SDIF_MODE = 0x00U;
	data->sdhi.p_reg->EXT_SWAP = 0x00U;

	clock_rate_set(&data->sdhi, ios->clock);

	switch (ios->bus_width)
	{
	case SDHC_BUS_WIDTH1BIT:
		data->sdhi.p_reg->SD_OPTION_b.WIDTH = 1;
		data->fsp_config.bus_width = SDMMC_BUS_WIDTH_1_BIT;
		break;

	case SDHC_BUS_WIDTH4BIT:
		data->sdhi.p_reg->SD_OPTION_b.WIDTH = 0;
		data->sdhi.p_reg->SD_OPTION_b.WIDTH8 = 0;
		data->fsp_config.bus_width = SDMMC_BUS_WIDTH_4_BITS;
		break;

	case SDHC_BUS_WIDTH8BIT:
		data->sdhi.p_reg->SD_OPTION_b.WIDTH = 0;
		data->sdhi.p_reg->SD_OPTION_b.WIDTH8 = 1;
		data->fsp_config.bus_width = SDMMC_BUS_WIDTH_8_BITS;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int sdhi_ra_get_card_present(const struct device *dev)
{
	struct sdhi_ra_data *data = dev->data;

	if (!dev || !dev->config) {
		return -EINVAL;
	}

	return (data->sdhi.p_reg->SD_INFO1_b.SDCDIN ||
			data->sdhi.p_reg->SD_INFO1_b.SDD3IN);
}


static int sdhi_ra_card_busy(const struct device *dev)
{
	struct sdhi_ra_data *data = dev->data;

	return (SDHI_PRV_SD_INFO2_CBSY_SDD0MON_IDLE_VAL !=
		(data->sdhi.p_reg->SD_INFO2 & SDHI_PRV_SD_INFO2_CBSY_SDD0MON_IDLE_MASK));

}

static int sdhi_ra_command_send(sdhi_instance_ctrl_t *sdhi, uint32_t command, uint32_t argument)
{
    /* Verify the device is not busy. */
    /* Wait for the device to stop holding DAT0 low. */
	while (SDHI_PRV_SD_INFO2_CBSY_SDD0MON_IDLE_VAL !=
		(sdhi->p_reg->SD_INFO2 & SDHI_PRV_SD_INFO2_CBSY_SDD0MON_IDLE_MASK))
	{
		k_sleep(K_MSEC(1));
	}

    /* Send the command. */
    /* Clear Status */
	sdhi->p_reg->SD_INFO1 = 0U;
	sdhi->p_reg->SD_INFO2 = 0U;
	sdhi->sdhi_event.word = 0U;

    /* Enable response end interrupt. */
    /* Disable access end interrupt and enable response end interrupt. */
	uint32_t mask = sdhi->p_reg->SD_INFO1_MASK;
    mask &= (~SDHI_PRV_SDHI_INFO1_RESPONSE_END);
    mask |= SDHI_PRV_SDHI_INFO1_ACCESS_END;
	sdhi->p_reg->SD_INFO1_MASK = mask;
	sdhi->p_reg->SD_INFO2_MASK = SDHI_PRV_SDHI_INFO2_MASK_CMD_SEND;

    /* Enable Clock */
	sdhi->p_reg->SD_CLK_CTRL |= SDHI_PRV_SDHI_PRV_SD_CLK_AUTO_CLOCK_ENABLE_MASK;

    /* Write argument, then command to the SDHI peripheral. */
	sdhi->p_reg->SD_ARG = argument & UINT16_MAX;
	sdhi->p_reg->SD_ARG1 = argument >> 16;
	sdhi->p_reg->SD_CMD = command;
}

static int sdhi_ra_transfer(const struct device *dev,
			struct sdhc_command *cmd,
			struct sdhc_data *data)
{
	struct sdhi_ra_data *dev_data = dev->data;

	switch (cmd->opcode)
	{
	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		/* dma write */
		break;
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
		/* dma read */
		break;
	default:
		break;
	}

	if (data->blocks > 1U) {
		dev_data->sdhi.p_reg->SD_STOP = SDHI_PRV_SD_STOP_SD_SECCNT_ENABLE;
		dev_data->sdhi.p_reg->SD_SECCNT = data->block_size;
	} else {
		dev_data->sdhi.p_reg->SD_STOP = 0U;
	}

	dev_data->sdhi.p_reg->SD_SIZE = data->block_size;

	sdhi_ra_command_send(&dev_data->sdhi, cmd->opcode, cmd->arg);
}

static void sdhi_ra_stop_transmission(const struct device *dev)
{
	struct sdhi_ra_data *dev_data = dev->data;

	LOG_WRN("Transfer failed, sending CMD12");

	sdhi_ra_command_send(&dev_data->sdhi, SD_STOP_TRANSMISSION, 0);
}
static int sdhi_ra_request(const struct device *dev,
			struct sdhc_command *cmd,
			struct sdhc_data *data)
{
	int ret;
	struct sdhi_ra_data *dev_data = dev->data;

	if (!dev || !cmd) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&dev_data->access_mutex, K_MSEC(cmd->timeout_ms));
	if (ret) {
		LOG_ERR("Could not access card");
		return -EBUSY;
	}

	k_sem_reset(&dev_data->transfer_sem);

	do {
		ret = sdhi_ra_transfer(dev, cmd, data);
		if (data && ret) {
			sdhi_ra_stop_transmission(dev);
			k_mutex_unlock(&dev_data->access_mutex);
		}

	} while (ret != 0 && (cmd->retries-- > 0));

	k_mutex_unlock(&dev_data->access_mutex);
	return ret;
}

static void sdhi_ra_init_host_props(const struct device *dev)
{
	struct sdhi_ra_data *data = dev->data;
	const struct sdhi_ra_config *config = dev->config;
	struct sdhc_host_props *props = &data->props;
	struct sdhc_host_caps *host_caps = &props->host_caps;

	memset(props, 0, sizeof(*props));

	/* Note: init only properties that are used for mmc/sdhc */

	props->f_max = config->max_frequency;

	props->f_min = 400 * 1000;
	// TODO: 确认最小频率

	props->power_delay = 100; /* ms */

	props->is_spi = false;

	host_caps->high_spd_support = 1;
	host_caps->vol_330_support = true;
	host_caps->vol_300_support = true;
	host_caps->vol_180_support = true;
	host_caps->bus_4_bit_support = true;
	host_caps->bus_8_bit_support = true;
}

static void ra8_sdhi_dmac_isr(const struct device *dev)
{
	dmac_int_isr();
}

static void ra8_sdhi_accs_isr(const struct device *dev)
{
	sdhimmc_accs_isr();
}

static void ra8_sdhi_card_isr(const struct device *dev)
{
	sdhimmc_card_isr();
}

static void ra8_sdhi_dma_req_isr(const struct device *dev)
{
	sdhimmc_dma_req_isr();
}

static void sdhi_dmac_callback (dmac_callback_args_t * p_args)
{
    r_sdhi_transfer_callback((sdhi_instance_ctrl_t *) p_args->p_context);
}

static void sdhi_callback(sdmmc_callback_args_t *p_args)
{
	struct sdhi_ra_data *data = (struct ra8_sdmmc_data*)p_args->p_context;
	switch (p_args->event)
	{
	case SDMMC_EVENT_TRANSFER_ERROR:
		break;
	case SDMMC_EVENT_ERASE_BUSY:
		break;
	case (SDMMC_EVENT_ERASE_COMPLETE | SDMMC_EVENT_TRANSFER_COMPLETE | SDMMC_EVENT_RESPONSE):
		k_sem_give(&data->transfer_sem);
	default:
		break;
	}

	return;
}

static int sdhi_ra_init(const struct device *dev)
{
	int ret = 0;
	fsp_err_t fsp_err;
	struct sdhi_ra_data *data = dev->data;
	struct sdhi_ra_config *config = dev->config;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	fsp_err = R_SDHI_Open(&data->sdhi, &data->fsp_config);
	__ASSERT(fsp_err == 0, "sdhi: initialization: fsp open failed");

	R_ICU->IELSR[14] = ELC_EVENT_DMAC0_INT;
	IRQ_CONNECT(14, 1, ra8_sdhi_dmac_isr, NULL, 0);

	sdhi_ra_init_host_props(dev);

	k_mutex_init(&data->access_mutex);
	k_sem_init(&data->transfer_sem, 0, 1);

	return 0;
}

static const struct sdhc_driver_api sdhi_api = {
	.reset = sdhi_ra_reset,
	.get_host_props = sdhi_ra_get_host_props,
	.set_io = sdhi_ra_set_io,
	.get_card_present = sdhi_ra_get_card_present,
	.request = sdhi_ra_request,
	.card_busy = sdhi_ra_card_busy,
};

#define _ELC_EVENT_SDHIMMC_ACCS(channel) ELC_EVENT_SDHIMMC##channel##_ACCS
#define _ELC_EVENT_SDHIMMC_CARD(channel) ELC_EVENT_SDHIMMC##channel##_CARD
#define _ELC_EVENT_SDHIMMC_DMA_REQ(channel) ELC_EVENT_SDHIMMC##channel##_DMA_REQ

#define ELC_EVENT_SDHIMMC_ACCS(channel) _ELC_EVENT_SDHIMMC_ACCS(channel)
#define ELC_EVENT_SDHIMMC_CARD(channel) _ELC_EVENT_SDHIMMC_CARD(channel)
#define ELC_EVENT_SDHIMMC_DMA_REQ(channel) _ELC_EVENT_SDHIMMC_DMA_REQ(channel)

#define SDHI_RA_INIT(n)                                                                    \
	static void sdhi_##n##_irq_config_func(const struct device *dev)                       \
	{                                                                                      \
		R_ICU->IELSR[DT_INST_IRQ_BY_NAME(n, accs, irq)] =                                  \
			ELC_EVENT_SDHIMMC_ACCS(DT_INST_PROP(n, channel));                              \
		R_ICU->IELSR[DT_INST_IRQ_BY_NAME(n, card, irq)] =                                  \
			ELC_EVENT_SDHIMMC_CARD(DT_INST_PROP(n, channel));                              \
		R_ICU->IELSR[DT_INST_IRQ_BY_NAME(n, dma_req, irq)] =                               \
			ELC_EVENT_SDHIMMC_DMA_REQ(DT_INST_PROP(n, channel));                           \
                                                                                           \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, accs, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, accs, priority), ra8_sdhi_accs_isr,                 \
			    DEVICE_DT_INST_GET(n), 0);                                                 \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, card, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, card, priority), ra8_sdhi_card_isr,                 \
			    DEVICE_DT_INST_GET(n), 0);                                                 \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, dma_req, irq),                                  \
			    DT_INST_IRQ_BY_NAME(n, dma_req, priority), ra8_sdhi_dma_req_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                                 \
	}                                                                                      \
                                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                             \
                                                                                           \
	static const struct sdhi_ra_config sdhi_##n##_config = {                               \
		.irq_config_func = sdhi_##n##_irq_config_func,                                     \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
	};                                                                                     \
                                                                                           \
	static struct sdhi_ra_data sdhi_##n##_data = {                                         \
		.fsp_config.channel = DT_INST_PROP(n, channel),                                    \
		.fsp_config.block_size = 512,                                                      \
		.fsp_config.card_detect = SDMMC_CARD_DETECT_CD,                                    \
		.fsp_config.write_protect = SDMMC_WRITE_PROTECT_NONE,                              \
		.fsp_config.p_extend = NULL,                                                       \
		.fsp_config.access_ipl = DT_INST_IRQ_BY_NAME(n, accs, priority),                   \
		.fsp_config.sdio_ipl = BSP_IRQ_DISABLED,                                           \
		.fsp_config.card_ipl = DT_INST_IRQ_BY_NAME(n, card, priority),                     \
		.fsp_config.dma_req_ipl = BSP_IRQ_DISABLED,                                        \
		.fsp_config.access_irq = DT_INST_IRQ_BY_NAME(n, accs, irq),                        \
		.fsp_config.card_irq = DT_INST_IRQ_BY_NAME(n, card, irq),                          \
		.fsp_config.sdio_irq = FSP_INVALID_VECTOR,                                         \
		.fsp_config.dma_req_irq = FSP_INVALID_VECTOR,                                      \
		.fsp_config.p_callback = sdhi_callback,                                            \
		.fsp_config.p_context = &sdhi_##n##_data,                                          \
		.fsp_config.p_lower_lvl_transfer = &sdhi_##n##_data.transfer,                      \
		.transfer.p_ctrl = &sdhi_##n##_data.transfer_ctrl,                                 \
		.transfer.p_cfg = &sdhi_##n##_data.transfer_cfg,                                   \
		.transfer.p_api = &g_transfer_on_dmac,                                             \
		.transfer_cfg.p_info = &sdhi_##n##_data.transfer_info,                             \
		.transfer_cfg.p_extend = &sdhi_##n##_data.transfer_extend,                         \
		.transfer_extend.offset = 1,                                                       \
		.transfer_extend.src_buffer_size = 1,                                              \
		.transfer_extend.irq = 14,                                                         \
		.transfer_extend.ipl = (12),                                                       \
		.transfer_extend.channel = 0,                                                      \
		.transfer_extend.p_callback = sdhi_dmac_callback,                                  \
		.transfer_extend.p_context = &sdhi_##n##_data.sdhi,                                \
		.transfer_extend.activation_source = ELC_EVENT_SDHIMMC1_DMA_REQ,                   \
		.transfer_info.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED, \
		.transfer_info.transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE, \
		.transfer_info.transfer_settings_word_b.irq = TRANSFER_IRQ_END,                    \
		.transfer_info.transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED, \
		.transfer_info.transfer_settings_word_b.src_addr_mode =                            \
			TRANSFER_ADDR_MODE_INCREMENTED,                                                \
		.transfer_info.transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE,               \
		.transfer_info.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,               \
		.transfer_info.p_dest = (void *)NULL,                                              \
		.transfer_info.p_src = (void const *)NULL,                                         \
		.transfer_info.num_blocks = 0,                                                     \
		.transfer_info.length = 128,                                                       \
	};                                                                                     \
                                                                                           \
	DEVICE_DT_INST_DEFINE(n, sdhi_ra_init, NULL, &sdhi_##n##_data, &sdhi_##n##_config,     \
			      POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, &sdhi_api);

DT_INST_FOREACH_STATUS_OKAY(SDHI_RA_INIT)

