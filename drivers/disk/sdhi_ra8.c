#define DT_DRV_COMPAT renesas_ra8_sdhi

#include <zephyr/devicetree.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <r_sdhi.h>
#include <r_dmac.h>

LOG_MODULE_REGISTER(ra_sdhi, CONFIG_SDMMC_LOG_LEVEL);

#define RM_BLOCK_MEDIA_SDMMC_PRV_SD_R1_ERRORS    (0xFDF98008U)


extern void r_sdhi_transfer_callback(sdhi_instance_ctrl_t *p_ctrl);
extern void sdhimmc_card_isr(void);
extern void sdhimmc_accs_isr(void);
extern void dmac_int_isr(void);

typedef void (*irq_config_func_t)(const struct device *dev);

struct ra_sdhi_data {
	irq_config_func_t irq_config;
	struct k_sem thread_lock;
	struct k_sem sync;
	int status;
	bool card_present;
	sdhi_instance_ctrl_t sdhi;
	sdmmc_cfg_t sdhi_cfg;
	sdmmc_api_t const * api;
	const transfer_instance_t transfer;
	dmac_instance_ctrl_t transfer_ctrl;
	const transfer_cfg_t transfer_cfg;
	const transfer_api_t transfer_api;
	transfer_info_t transfer_info;
	const dmac_extended_cfg_t transfer_extend;
	sdmmc_device_t device;
	const struct pinctrl_dev_config *pcfg;
};


static int ra_sdhi_access_init(struct disk_info *disk)
{
	struct ra_sdhi_data *data = disk->dev->data;
	fsp_err_t fsp_err = FSP_ERR_INVALID_HW_CONDITION;
	sdmmc_status_t status;
	uint64_t memory_size_mb;

	if (data->sdhi.initialized == false) {
		fsp_err = data->api->mediaInit(&data->sdhi, &data->device);
		if (fsp_err != FSP_SUCCESS) {
			LOG_ERR("failed to initialize sd/sdmmc device: %d", fsp_err);
			return -EIO;
		}
	}

	fsp_err = data->api->statusGet(&data->sdhi, &status);
	if (fsp_err != FSP_SUCCESS)
	{
		LOG_ERR("failed to get sd/sdmmc device status: %d", fsp_err);
		return -EIO;
	}

	// data->card_present = (status.card_inserted == true) ? true : false;
	data->card_present = true;
	data->status = DISK_STATUS_OK;

	memory_size_mb = (uint64_t)data->device.sector_count * data->device.sector_size_bytes;

	LOG_INF("clock rate:  %u MHz", data->device.clock_rate / 1000000);
	LOG_INF("sector count: %u", data->device.sector_count);
	LOG_INF("sector size: %u", data->device.sector_size_bytes);
	LOG_INF("Minimum erasable unit (in 512 byte sectors): %u", data->device.erase_sector_count);
	LOG_INF("Memory Size: %u MB", (uint32_t)(memory_size_mb >> 20));

	return 0;
}

static int ra_sdhi_access_status(struct disk_info *disk)
{
	struct ra_sdhi_data *data = disk->dev->data;

	return data->status;
}

static int ra_sdhi_access_read(struct disk_info *disk, uint8_t *data_buf, uint32_t start_sector,
			       uint32_t num_sector)
{
	struct ra_sdhi_data *data = (struct ra_sdhi_data *)disk->dev->data;
	fsp_err_t fsp_err;
	int err = 0;
	sdmmc_status_t status;

	k_sem_take(&data->thread_lock, K_FOREVER);

	do {
		data->api->statusGet(&data->sdhi, &status);
	} while (status.transfer_in_progress);

	fsp_err = data->api->read(&data->sdhi, data_buf, start_sector, num_sector);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("sdhi read failed: %d", fsp_err);
		err = -EIO;
		goto end;
	}

	k_sem_take(&data->sync, K_FOREVER);

	if (data->status != DISK_STATUS_OK) {
		LOG_ERR("sd read error %d", data->status);
		err = -EIO;
		goto end;
	}

	do {
		data->api->statusGet(&data->sdhi, &status);
	} while (status.transfer_in_progress);

end:
	k_sem_give(&data->thread_lock);
	return err;
}

static int ra_sdhi_access_write(struct disk_info *disk, const uint8_t *data_buf,
				uint32_t start_sector, uint32_t num_sector)
{
	struct ra_sdhi_data *data = (struct ra_sdhi_data *)disk->dev->data;
	fsp_err_t fsp_err;
	int err = 0;
	sdmmc_status_t status;

	k_sem_take(&data->thread_lock, K_FOREVER);

	do {
		data->api->statusGet(&data->sdhi, &status);
	} while (status.transfer_in_progress);

	fsp_err = data->api->write(&data->sdhi, data_buf, start_sector, num_sector);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("sdhi write failed: %d", fsp_err);
		err = -EIO;
		goto end;
	}

	k_sem_take(&data->sync, K_FOREVER);

	if (data->status != DISK_STATUS_OK) {
		LOG_ERR("sd write error %d", data->status);
		err = -EIO;
		goto end;
	}

	do {
		data->api->statusGet(&data->sdhi, &status);
	} while (status.transfer_in_progress);

end:
	k_sem_give(&data->thread_lock);
	return err;
}

static int ra_sdhi_access_ioctl(struct disk_info *disk, uint8_t cmd,
				    void *buff)
{
	struct ra_sdhi_data *data = (struct ra_sdhi_data*)disk->dev->data;

	switch (cmd)
	{
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t*)buff = data->device.sector_count;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t*)buff = data->device.sector_size_bytes;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t*)buff = data->device.erase_sector_count;
	case DISK_IOCTL_CTRL_SYNC:
		/* we use a blocking API, so nothing to do for sync */
		break;
	case DISK_IOCTL_CTRL_INIT:
		return ra_sdhi_access_init(disk);
	case DISK_IOCTL_CTRL_DEINIT:
		data->status = DISK_STATUS_UNINIT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct disk_operations sdhi_ops = {
	.init = ra_sdhi_access_init,
	.status = ra_sdhi_access_status,
	.read = ra_sdhi_access_read,
	.write = ra_sdhi_access_write,
	.ioctl = ra_sdhi_access_ioctl,
};

static struct disk_info sdhi_info = {
	.name = CONFIG_SDMMC_VOLUME_NAME,
	.ops = &sdhi_ops,
};

static void ra_sdhi_card_isr(const struct device *dev)
{
	sdhimmc_card_isr();
}

static void ra_sdhi_access_isr(const struct device *dev)
{
	sdhimmc_accs_isr();
}

static void ra_sdhi_dmac_isr(const struct device *dev)
{
	dmac_int_isr();
}

static void ra_sdhi_irq_config_func(const struct device *dev)
{
	struct ra_sdhi_data *data = (struct ra_sdhi_data *)dev->data;

	R_ICU->IELSR[data->sdhi_cfg.card_irq] = ELC_EVENT_SDHIMMC1_CARD;
	R_ICU->IELSR[data->sdhi_cfg.access_irq] = ELC_EVENT_SDHIMMC1_ACCS;
	R_ICU->IELSR[data->transfer_extend.irq] = ELC_EVENT_DMAC1_INT;

	IRQ_CONNECT(1U, 12U, ra_sdhi_card_isr, DEVICE_DT_INST_GET(0), 0);
	IRQ_CONNECT(0U, 12, ra_sdhi_access_isr, DEVICE_DT_INST_GET(0), 0);
	IRQ_CONNECT(2U, 12, ra_sdhi_dmac_isr, DEVICE_DT_INST_GET(0), 0);
}

static int disk_ra_sdhi_init(const struct device *dev)
{
	struct ra_sdhi_data *data = dev->data;
	int err;

	err = pinctrl_apply_state(data->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("failed to set pin status");
		return err;
	}

	data->irq_config(dev);

	/* Initialize semaphores */
	k_sem_init(&data->thread_lock, 1, 1);
	k_sem_init(&data->sync, 0, 1);

	data->api->open(&data->sdhi, &data->sdhi_cfg);

	data->card_present = true;

	sdhi_info.dev = dev;
	err = disk_access_register(&sdhi_info);
	if (err) {
		data->api->close(&data->sdhi);
		return err;
	}

	return 0;
}

void dmac_callback(dmac_callback_args_t *p_args)
{
	r_sdhi_transfer_callback ((sdhi_instance_ctrl_t*) p_args->p_context);
}

void sdhi_callback(sdmmc_callback_args_t *p_args)
{
	struct ra_sdhi_data *data = (struct ra_sdhi_data *)p_args->p_context;

	if ((SDMMC_EVENT_ERASE_COMPLETE | SDMMC_EVENT_TRANSFER_COMPLETE) & p_args->event) {
		k_sem_give(&data->sync);
		return;
	}

	if ((SDMMC_EVENT_TRANSFER_ERROR & p_args->event) ||
	    (data->sdhi.initialized &&
	     (RM_BLOCK_MEDIA_SDMMC_PRV_SD_R1_ERRORS & p_args->response.status))) {
		data->status = (RM_BLOCK_MEDIA_SDMMC_PRV_SD_R1_ERRORS & p_args->response.status);
		k_sem_give(&data->sync);
		return;
	}

	if (SDMMC_EVENT_ERASE_BUSY & p_args->event) {
		return;
	}

	if (SDMMC_EVENT_CARD_REMOVED & p_args->event) {
		data->card_present = false;
		return;
	}

	if (SDMMC_EVENT_CARD_INSERTED & p_args->event) {
		data->card_present = true;
		return;
	}
}

PINCTRL_DT_INST_DEFINE(0);

static struct ra_sdhi_data ra_sdhi_priv = {
	.irq_config = ra_sdhi_irq_config_func,
	.sdhi_cfg = {
		.channel = 1,
		.bus_width = SDMMC_BUS_WIDTH_4_BITS,
		.p_callback = sdhi_callback,
		.p_context = (const void*)&ra_sdhi_priv,
		.block_size = 512,
		.card_detect = SDMMC_CARD_DETECT_CD,
		.write_protect = SDMMC_WRITE_PROTECT_NONE,
		.p_extend = NULL,
		.p_lower_lvl_transfer = &ra_sdhi_priv.transfer,
		.access_ipl = (12),
		.sdio_ipl = (0xFFU),
		.card_ipl = (12),
		.dma_req_ipl = (0xFFU),
		.access_irq = ((IRQn_Type) 0),
		.card_irq = ((IRQn_Type) 1),
		.sdio_irq = ((IRQn_Type) - 33),
		.dma_req_irq = ((IRQn_Type) - 33),
	},
	.api = &g_sdmmc_on_sdhi,
	.transfer = {
		.p_ctrl = &ra_sdhi_priv.transfer_ctrl,
		.p_cfg = &ra_sdhi_priv.transfer_cfg,
		.p_api = &g_transfer_on_dmac,
	},
	.transfer_cfg = {
		.p_info = &ra_sdhi_priv.transfer_info,
		.p_extend = &ra_sdhi_priv.transfer_extend,
	},
	.transfer_extend = {
		.offset = 1,
		.src_buffer_size = 1,
		.irq = ((IRQn_Type) 2),
		.ipl = (12),
		.channel = 1,
		.p_callback = dmac_callback,
		.p_context = &ra_sdhi_priv.sdhi,
		.activation_source = ELC_EVENT_SDHIMMC1_DMA_REQ,
	},
	.transfer_info = {
		.transfer_settings_word_b = {
			.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
			.repeat_area = TRANSFER_REPEAT_AREA_SOURCE,
			.irq = TRANSFER_IRQ_END,
			.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
			.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
			.size = TRANSFER_SIZE_4_BYTE,
			.mode = TRANSFER_MODE_NORMAL,
		},
		.p_dest = (void*) NULL,
		.p_src = (void const*) NULL,
		.num_blocks = 0,
		.length = 128,
	},
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};


DEVICE_DT_INST_DEFINE(0, disk_ra_sdhi_init, NULL,
		    &ra_sdhi_priv, NULL, POST_KERNEL,
		    CONFIG_SD_INIT_PRIORITY,
		    NULL);

