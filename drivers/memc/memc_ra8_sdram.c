/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra8_bus_sdram

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(memc_ra8_sdram, CONFIG_MEMC_LOG_LEVEL);

#define BSP_PRV_SDRAM_CL                       (3U)

#define BSP_PRV_SDRAM_MR_WB_SINGLE_LOC_ACC    (1U) /* MR.M9                : Single Location Access */
#define BSP_PRV_SDRAM_MR_OP_MODE              (0U) /* MR.M8:M7             : Standard Operation */
#define BSP_PRV_SDRAM_MR_BT_SEQUENCTIAL       (0U) /* MR.M3 Burst Type     : Sequential */
#define BSP_PRV_SDRAM_MR_BURST_LENGTH         (0U) /* MR.M2:M0 Burst Length: 0(1 burst) */

typedef struct {
	uint32_t SDBank; /*!< Specifies the SDRAM memory device that will be used.
			      This parameter can be a value of @ref FMC_SDRAM_Bank                */

	uint32_t ColumnBitsNumber; /*!< Defines the number of bits of column address.
					This parameter can be a value of @ref
				      FMC_SDRAM_Column_Bits_number. */

	uint32_t RowBitsNumber; /*!< Defines the number of bits of column address.
				     This parameter can be a value of @ref
				   FMC_SDRAM_Row_Bits_number.    */

	uint32_t MemoryDataWidth; /*!< Defines the memory device width.
				       This parameter can be a value of @ref
				     FMC_SDRAM_Memory_Bus_Width.   */

	uint32_t InternalBankNumber; /*!< Defines the number of the device's internal banks.
					  This parameter can be of @ref
					FMC_SDRAM_Internal_Banks_Number.      */

	uint32_t CASLatency; /*!< Defines the SDRAM CAS latency in number of memory clock cycles.
				  This parameter can be a value of @ref FMC_SDRAM_CAS_Latency. */

	uint32_t WriteProtection; /*!< Enables the SDRAM device to be accessed in write mode.
				       This parameter can be a value of @ref
				     FMC_SDRAM_Write_Protection.   */

	uint32_t SDClockPeriod; /*!< Define the SDRAM Clock Period for both SDRAM devices and they
				   allow to disable the clock before changing frequency. This
				   parameter can be a value of @ref FMC_SDRAM_Clock_Period.       */

	uint32_t ReadBurst; /*!< This bit enable the SDRAM controller to anticipate the next read
				 commands during the CAS latency and stores data in the Read FIFO.
				 This parameter can be a value of @ref FMC_SDRAM_Read_Burst. */

	uint32_t ReadPipeDelay; /*!< Define the delay in system clock cycles on read data path.
				     This parameter can be a value of @ref
				   FMC_SDRAM_Read_Pipe_Delay.    */
} RA8_SDRAM_InitTypeDef;

typedef struct {
	uint32_t LoadToActiveDelay; /*!< Defines the delay between a Load Mode Register command and
					 an active or Refresh command in number of memory clock
				       cycles. This parameter can be a value between Min_Data = 1
				       and Max_Data = 16  */

	uint32_t ExitSelfRefreshDelay; /*!< Defines the delay from releasing the self refresh
					  command to issuing the Activate command in number of
					  memory clock cycles. This parameter can be a value between
					  Min_Data = 1 and Max_Data = 16  */

	uint32_t SelfRefreshTime; /*!< Defines the minimum Self Refresh period in number of memory
				     clock cycles. This parameter can be a value between Min_Data =
				     1 and Max_Data = 16  */

	uint32_t RowCycleDelay; /*!< Defines the delay between the Refresh command and the Activate
				   command and the delay between two consecutive Refresh commands in
				   number of memory clock cycles. This parameter can be a value
				   between Min_Data = 1 and Max_Data = 16  */

	uint32_t WriteRecoveryTime; /*!< Defines the Write recovery Time in number of memory clock
				       cycles. This parameter can be a value between Min_Data = 1
				       and Max_Data = 16  */

	uint32_t RPDelay; /*!< Defines the delay between a Precharge Command and an other command
			       in number of memory clock cycles.
			       This parameter can be a value between Min_Data = 1 and Max_Data = 16
			   */

	uint32_t RCDDelay; /*!< Defines the delay between the Activate Command and a Read/Write
				command in number of memory clock cycles.
				This parameter can be a value between Min_Data = 1 and Max_Data = 16
			    */
} RA8_SDRAM_TimingTypeDef;

struct memc_ra8_sdram_bank_config {
	RA8_SDRAM_InitTypeDef init;
	RA8_SDRAM_TimingTypeDef timing;
};

struct memc_ra8_sdram_config {
	const struct pinctrl_dev_config *pcfg;
	R_BUS_Type *bus;
	uint32_t power_up_delay;
	uint8_t num_auto_refresh;
	uint16_t mode_register;
	uint16_t refresh_rate;
	const struct memc_ra8_sdram_bank_config *banks;
	size_t banks_len;
};

static int memc_ra8_sdram_init(const struct device *dev)
{
	const struct memc_ra8_sdram_config *config = dev->config;
	R_BUS_Type *bus = config->bus;

	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (config->banks->timing.RPDelay < 3U) {
		bus->SDRAM.SDIR_b.PRC = config->banks->timing.RPDelay;
	} else {
		bus->SDRAM.SDIR_b.PRC = config->banks->timing.RCDDelay - 3U;
	}

	while (bus->SDRAM.SDSR) {
		/* code */
	}

	bus->SDRAM.SDIR_b.ARFC = 2U;

	while (bus->SDRAM.SDSR) {
		/* code */
	}

	if (config->num_auto_refresh < 3) {
		bus->SDRAM.SDIR_b.ARFI = 0U;
	} else {
		bus->SDRAM.SDIR_b.ARFI = config->num_auto_refresh;
	}

	while (bus->SDRAM.SDSR) {
		/* code */
	}

	bus->SDRAM.SDICR_b.INIRQ = 1U;

	while (bus->SDRAM.SDSR_b.INIST) {
		/* Wait the end of initialization sequence. */
	}

	bus->SDRAM.SDCCR_b.BSIZE = 0;
	bus->SDRAM.SDAMOD_b.BE = 1;
	bus->SDRAM.SDCMOD_b.EMODE = 0;

	while (bus->SDRAM.SDSR) {
		/* code */
	}

    /** Using LMR command, program the mode register */
    bus->SDRAM.SDMOD = ((((uint16_t)(BSP_PRV_SDRAM_MR_WB_SINGLE_LOC_ACC << 9) |
                            (uint16_t)(BSP_PRV_SDRAM_MR_OP_MODE << 7)) |
                           (uint16_t)(BSP_PRV_SDRAM_CL << 4)) |
                          (uint16_t)(BSP_PRV_SDRAM_MR_BT_SEQUENCTIAL << 3)) |
                         (uint16_t)(BSP_PRV_SDRAM_MR_BURST_LENGTH << 0);

    /** wait at least tMRD time */
    while (bus->SDRAM.SDSR_b.MRSST)
    {
        /* Wait until Mode Register setting done. */
    }

	bus->SDRAM.SDTR_b.RAS = config->banks->timing.SelfRefreshTime - 1U;
	bus->SDRAM.SDTR_b.RCD = config->banks->timing.RCDDelay - 1U;
	bus->SDRAM.SDTR_b.RP = config->banks->timing.RPDelay -1U;
	bus->SDRAM.SDTR_b.WR = config->banks->timing.WriteRecoveryTime - 1U;
	bus->SDRAM.SDTR_b.CL = 3U;

	bus->SDRAM.SDADR_b.MXC = 1U;

	bus->SDRAM.SDRFCR_b.REFW = config->num_auto_refresh -1;
	bus->SDRAM.SDRFCR_b.RFC = 936U;

    /** Start Auto-refresh */
    R_BUS->SDRAM.SDRFEN_b.RFEN = 1U;

    /** Enable SDRAM access */
    R_BUS->SDRAM.SDCCR_b.EXENB = 1U;

	return 0;
}

/** SDRAM bank/s configuration initialization macro. */
#define BANK_CONFIG(node_id)                                                                       \
	{.init =                                                                                   \
		 {                                                                                 \
			 .SDBank = DT_REG_ADDR(node_id),                                           \
			 .ColumnBitsNumber = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 0),        \
			 .RowBitsNumber = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 1),           \
			 .MemoryDataWidth = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 2),         \
			 .InternalBankNumber = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 3),      \
			 .CASLatency = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 4),              \
			 .WriteProtection = 0,                                                     \
			 .SDClockPeriod = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 5),           \
			 .ReadBurst = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 6),               \
			 .ReadPipeDelay = DT_PROP_BY_IDX(node_id, ra8_sdram_control, 7),           \
		 },                                                                                \
	 .timing = {                                                                               \
		 .LoadToActiveDelay = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 0),                \
		 .ExitSelfRefreshDelay = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 1),             \
		 .SelfRefreshTime = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 2),                  \
		 .RowCycleDelay = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 3),                    \
		 .WriteRecoveryTime = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 4),                \
		 .RPDelay = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 5),                          \
		 .RCDDelay = DT_PROP_BY_IDX(node_id, ra8_sdram_timing, 6),                         \
	 }},

static const struct memc_ra8_sdram_bank_config bank_config[] = {
	DT_INST_FOREACH_CHILD(0, BANK_CONFIG)};

PINCTRL_DT_INST_DEFINE(0);

/** SDRAM configuration. */
static const struct memc_ra8_sdram_config config = {
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.bus = (R_BUS_Type *)(DT_REG_ADDR(DT_INST_PARENT(0))),
	.power_up_delay = DT_INST_PROP(0, power_up_delay),
	.num_auto_refresh = DT_INST_PROP(0, num_auto_refresh),
	.mode_register = DT_INST_PROP(0, mode_register),
	.refresh_rate = DT_INST_PROP(0, refresh_rate),
	.banks = bank_config,
	.banks_len = ARRAY_SIZE(bank_config),
};

DEVICE_DT_INST_DEFINE(0, memc_ra8_sdram_init, NULL, NULL, &config, POST_KERNEL,
		      CONFIG_MEMC_INIT_PRIORITY, NULL);
