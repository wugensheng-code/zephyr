/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RA_H_

#define MSTPA 0
#define MSTPB 1
#define MSTPC 2
#define MSTPD 3
#define MSTPE 4

/* LCD clock divider options. */
#define RA_LCD_CLOCK_DIV_1 0
#define RA_LCD_CLOCK_DIV_2 1
#define RA_LCD_CLOCK_DIV_3 5
#define RA_LCD_CLOCK_DIV_4 2
#define RA_LCD_CLOCK_DIV_5 6
#define RA_LCD_CLOCK_DIV_6 3
#define RA_LCD_CLOCK_DIV_8 4

#define MSTPA 0x40203000
#define MSTPB 0x40203004
#define MSTPC 0x40203008
#define MSTPD 0x4020300C
#define MSTPE 0x40203010

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RA_H_ */
