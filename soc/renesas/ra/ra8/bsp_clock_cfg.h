/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>

#ifndef BSP_CLOCK_CFG_H_
#define BSP_CLOCK_CFG_H_

#define BSP_CFG_CLOCKS_SECURE   (0)
#define BSP_CFG_CLOCKS_OVERRIDE (0)

#define BSP_CFG_XTAL_HZ DT_PROP(DT_NODELABEL(xtal), clock_frequency)

#if DT_PROP(DT_NODELABEL(hoco), clock_frequency) == 16000000
#define BSP_CFG_HOCO_FREQUENCY 0 /* HOCO 16MHz */
#elif DT_PROP(DT_NODELABEL(hoco), clock_frequency) == 18000000
#define BSP_CFG_HOCO_FREQUENCY 1 /* HOCO 18MHz */
#elif DT_PROP(DT_NODELABEL(hoco), clock_frequency) == 20000000
#define BSP_CFG_HOCO_FREQUENCY 2 /* HOCO 20MHz */
#elif DT_PROP(DT_NODELABEL(hoco), clock_frequency) == 32000000
#define BSP_CFG_HOCO_FREQUENCY 4 /* HOCO 32MHz */
#elif DT_PROP(DT_NODELABEL(hoco), clock_frequency) == 48000000
#define BSP_CFG_HOCO_FREQUENCY 7 /* HOCO 48MHz */
#else
#error "Invalid HOCO frequency, only can be set to 16MHz, 18MHz, 20MHz, 32MHz, 48MHz"
#endif

#define BSP_CFG_PLL_SOURCE DT_PROP(DT_NODELABEL(pll), source)
#define BSP_CFG_PLL_DIV    DT_PROP(DT_NODELABEL(pll), div)
#define BSP_CFG_PLL_MUL                                                                            \
	BSP_CLOCKS_PLL_MUL(DT_PROP_BY_IDX(DT_NODELABEL(pll), mul, 0),                        \
			   DT_PROP_BY_IDX(DT_NODELABEL(pll), mul, 1))

#define BSP_CFG_PLODIVP            DT_PROP(DT_NODELABEL(pll), divp)
#define BSP_CFG_PLL1P_FREQUENCY_HZ DT_PROP(DT_NODELABEL(pll), freqp)
#define BSP_CFG_PLODIVQ            DT_PROP(DT_NODELABEL(pll), divq)
#define BSP_CFG_PLL1Q_FREQUENCY_HZ DT_PROP(DT_NODELABEL(pll), freqq)
#define BSP_CFG_PLODIVR            DT_PROP(DT_NODELABEL(pll), divr)
#define BSP_CFG_PLL1R_FREQUENCY_HZ DT_PROP(DT_NODELABEL(pll), freqr)

#define BSP_CFG_PLL2_SOURCE DT_PROP(DT_NODELABEL(pll2), source)
#define BSP_CFG_PLL2_DIV    DT_PROP(DT_NODELABEL(pll2), div)
#define BSP_CFG_PLL2_MUL                                                                           \
	BSP_CLOCKS_PLL_MUL(DT_PROP_BY_IDX(DT_NODELABEL(pll2), mul, 0),                       \
			   DT_PROP_BY_IDX(DT_NODELABEL(pll2), mul, 1))

#define BSP_CFG_PL2ODIVP           DT_PROP(DT_NODELABEL(pll2), divp)
#define BSP_CFG_PLL2P_FREQUENCY_HZ DT_PROP(DT_NODELABEL(pll2), freqp)
#define BSP_CFG_PL2ODIVQ           DT_PROP(DT_NODELABEL(pll2), divq)
#define BSP_CFG_PLL2Q_FREQUENCY_HZ DT_PROP(DT_NODELABEL(pll2), freqq)
#define BSP_CFG_PL2ODIVR           DT_PROP(DT_NODELABEL(pll2), divr)
#define BSP_CFG_PLL2R_FREQUENCY_HZ DT_PROP(DT_NODELABEL(pll2), freqr)

#define BSP_CFG_CLOCK_SOURCE DT_PROP(DT_NODELABEL(cpuclk), clk_src)
#define BSP_CFG_CPUCLK_DIV   DT_PROP(DT_NODELABEL(cpuclk), clk_div)

#define BSP_CFG_ICLK_DIV     DT_PROP(DT_NODELABEL(iclk), clk_div)
#define BSP_CFG_PCLKA_DIV    DT_PROP(DT_NODELABEL(pclka), clk_div)
#define BSP_CFG_PCLKB_DIV    DT_PROP(DT_NODELABEL(pclkb), clk_div)
#define BSP_CFG_PCLKC_DIV    DT_PROP(DT_NODELABEL(pclkc), clk_div)
#define BSP_CFG_PCLKD_DIV    DT_PROP(DT_NODELABEL(pclkd), clk_div)
#define BSP_CFG_PCLKE_DIV    DT_PROP(DT_NODELABEL(pclke), clk_div)
#define BSP_CFG_BCLK_DIV     DT_PROP(DT_NODELABEL(bclk), clk_div)
#define BSP_CFG_BCLK_OUTPUT  DT_PROP(DT_NODELABEL(bclk), clk_out)
#define BSP_CFG_FCLK_DIV     DT_PROP(DT_NODELABEL(fclk), clk_div)

#define BSP_CFG_UCK_SOURCE      DT_PROP(DT_NODELABEL(uclk), clk_src)
#define BSP_CFG_UCK_DIV         DT_PROP(DT_NODELABEL(uclk), clk_div)
#define BSP_CFG_U60CK_SOURCE    DT_PROP(DT_NODELABEL(u60clk), clk_src)
#define BSP_CFG_U60CK_DIV       DT_PROP(DT_NODELABEL(u60clk), clk_div)
#define BSP_CFG_OCTA_SOURCE     DT_PROP(DT_NODELABEL(octaspiclk), clk_src)
#define BSP_CFG_OCTA_DIV        DT_PROP(DT_NODELABEL(octaspiclk), clk_div)
#define BSP_CFG_CANFDCLK_SOURCE DT_PROP(DT_NODELABEL(canfdclk), clk_src)
#define BSP_CFG_CANFDCLK_DIV    DT_PROP(DT_NODELABEL(canfdclk), clk_div)
#define BSP_CFG_CLKOUT_SOURCE   DT_PROP(DT_NODELABEL(clkout), clk_src)
#define BSP_CFG_CLKOUT_DIV      DT_PROP(DT_NODELABEL(clkout), clk_div)
#define BSP_CFG_SCICLK_SOURCE   DT_PROP(DT_NODELABEL(sciclk), clk_src)
#define BSP_CFG_SCICLK_DIV      DT_PROP(DT_NODELABEL(sciclk), clk_div)
#define BSP_CFG_SPICLK_SOURCE   DT_PROP(DT_NODELABEL(spiclk), clk_src)
#define BSP_CFG_SPICLK_DIV      DT_PROP(DT_NODELABEL(spiclk), clk_div)
#define BSP_CFG_ADCCLK_SOURCE   DT_PROP(DT_NODELABEL(adcclk), clk_src)
#define BSP_CFG_ADCCLK_DIV      DT_PROP(DT_NODELABEL(adcclk), clk_div)
#define BSP_CFG_I3CCLK_SOURCE   DT_PROP(DT_NODELABEL(i3cclk), clk_src)
#define BSP_CFG_I3CCLK_DIV      DT_PROP(DT_NODELABEL(i3cclk), clk_div)
#define BSP_CFG_LCDCLK_SOURCE   DT_PROP(DT_NODELABEL(lcdclk), clk_src)
#define BSP_CFG_LCDCLK_DIV      DT_PROP(DT_NODELABEL(lcdclk), clk_div)

#endif /* BSP_CLOCK_CFG_H_ */
