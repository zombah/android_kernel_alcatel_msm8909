/*
* arch/arm/mach-msm/include/mach/watchdogtest.h: include file for watchdog
*
* Copyright (C) chenghui.jia@tcl.com.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation.
*
* add by jch for watchdog reset test PR-802266
*/

#ifndef WATCHDOGTEST_H
#define WATCHDOGTEST_H
#ifdef CONFIG_JRD_BUTTON_RAMCONSOLE_WDT
#include "../../msm_watchdog.h"

static inline void touch_hw_watchdog(void)
{
   	g_pet_watchdog();
}
   
   static inline void trigger_watchdog_reset(void)
{
	msm_watchdog_reset(0);
}
#endif
#endif
