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
* v0.1
* add Galaxy S6 Under- and Overclock
*
*
*
*/

/* Samsung Galaxy S6 G92X EXYNOS 7420
*   apollo cluster (little core)
* 	L0 =	2000MHz
*	L1 =	1900MHz
*	L2 =	1800MHz
*	L3 =	1700MHz
*	L4 =	1600MHz
*	L5 =	1500MHz
*	L6 =	1400MHz
*	L7 =	1300MHz
*	L8 =	1200MHz
*	L9 =	1100MHz
*	L10 =	1000MHz
*	L11 =	900MHz
*	L12 =	800MHz
*	L13 =	700MHz
*	L14 =	600MHz
*	L15 =	500MHz
*	L16 =	400MHz
*	L17 =	300MHz
*	L18 =	200MHz
*
*   atlas cluster (big core)
*	L0 =	2500MHz
*	L1 =	2400MHz
*	L2 =	2300MHz
*	L3 =	2200MHz
*	L4 =	2100MHz
*	L5 =	2000MHz
*	L6 =	1900MHz
*	L7 =	1800MHz
*	L8 =	1700MHz
*	L9 =	1600MHz
*	L10 =	1500MHz
*	L11 =	1400MHz
*	L12 =	1300MHz
*	L13 =	1200MHz
*	L14 =	1100MHz
*	L15 =	1000MHz
*	L16 =	900MHz
*	L17 =	800MHz
*	L18 =	700MHz
*	L19 =	600MHz
*	L20 =	500MHz
*	L21 =	400MHz
*	L22 =	300MHz
*	L23 =	200MHz
*
*/

#include <linux/cpufreq.h>

/* CPU*/

#ifdef CONFIG_G92X_CPU_UNDERCLOCK
	#define G92X_CPU_MIN_FREQ_LITTLE L18 		/* 200MHz */
	#define G92X_CPU_MIN_FREQ_BIG L23 		/* 200MHz */
#else
 	#define G92X_CPU_MIN_FREQ_LITTLE L16 		/* 400MHz */
	#define G92X_CPU_MIN_FREQ_BIG L17		/* 800MHz */
#endif

#ifdef CONFIG_G92X_CPU_OVERCLOCK
	#define G92X_CPU_MAX_FREQ_LITTLE L4		/* 1600MHz */
	#define G92X_CPU_MAX_FREQ_BIG L0		/* 2500MHz */
#else
 	#define G92X_CPU_MAX_FREQ_LITTLE L5		/* 1500MHz */
	#define G92X_CPU_MAX_FREQ_BIG L4		/* 2100MHz */
#endif

