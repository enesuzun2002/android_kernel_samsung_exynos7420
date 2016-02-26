/*
 * SAMSUNG EXYNOS UNIVERSAL CLOCKER
 * <clocker.h>
 * Copyright (c) 2015 thehacker911 <maikdiebenkorn@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/*CHANGELOG:
*
*
*
*
*/

/* Samsung Galaxy S6 G92X EXYNOS 7420
*   apollo cluster (little core)
* 	{L0,  2000 * 1000},
*	{L1,  1900 * 1000},
*	{L2,  1800 * 1000},
*	{L3,  1704 * 1000},
*	{L4,  1600 * 1000},
*	{L5,  1500 * 1000},
*	{L6,  1400 * 1000},
*	{L7,  1296 * 1000},
*	{L8,  1200 * 1000},
*	{L9,  1104 * 1000},
*	{L10, 1000 * 1000},
*	{L11,  900 * 1000},
*	{L12,  800 * 1000},
*	{L13,  700 * 1000},
*	{L14,  600 * 1000},
*	{L15,  500 * 1000},
*	{L16,  400 * 1000},
*	{L17,  300 * 1000},
*	{L18,  200 * 1000},
*
*   atlas cluster (big core)
*	{L0,  2496 * 1000},
*	{L1,  2400 * 1000},
*	{L2,  2304 * 1000},
*	{L3,  2200 * 1000},
*	{L4,  2100 * 1000},
*	{L5,  2000 * 1000},
*	{L6,  1896 * 1000},
*	{L7,  1800 * 1000},
*	{L8,  1704 * 1000},
*	{L9,  1600 * 1000},
*	{L10, 1500 * 1000},
*	{L11, 1400 * 1000},
*	{L12, 1300 * 1000},
*	{L13, 1200 * 1000},
*	{L14, 1100 * 1000},
*	{L15, 1000 * 1000},
*	{L16,  900 * 1000},
*	{L17,  800 * 1000},
*	{L18,  700 * 1000},
*	{L19,  600 * 1000},
*	{L20,  500 * 1000},
*	{L21,  400 * 1000},
*	{L22,  300 * 1000},
*	{L23,  200 * 1000},
*
*/

#include <linux/cpufreq.h>

#ifdef CONFIG_G92X_CPU_UNDERCLOCK
	#define G92X_CPU_MIN_FREQ_LITTLE L18; 		/* 200MHz */
	#define G92X_CPU_MIN_FREQ_BIG L23; 		/* 200MHz */
#else
 	#define G92X_CPU_MIN_FREQ_LITTLE L16; 		/* 400MHz */
	#define G92X_CPU_MIN_FREQ_BIG L17; 		/* 800MHz */
#endif

#ifdef CONFIG_G92X_CPU_OVERCLOCK
	#define G92X_CPU_MAX_FREQ_LITTLE L4;		/* 1600MHz */
	#define G92X_CPU_MAX_FREQ_BIG L0;		/* 2500MHz */
#else
 	#define G92X_CPU_MAX_FREQ_LITTLE L5;		/* 1500MHz */
	#define G92X_CPU_MAX_FREQ_BIG L4;		/* 2100MHz */
#endif




