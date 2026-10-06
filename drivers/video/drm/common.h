/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _DRM_COMPAT_COMMON_H_
#define _DRM_COMPAT_COMMON_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/io.h>
#include <dm/device.h>
#include <dm/ofnode.h>
#include <dm/devres.h>
#include <vsprintf.h>
#include <env.h>
#include <time.h>
#include <command.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <bmp_layout.h>
#include <display_options.h>
#include <linux/bug.h>

#define DM_GET_DRIVER(name) DM_DRIVER_GET(name)

#define get_timer_masked() get_timer(0)

static inline uint get_hosc(void)
{
	return 24000000;
}

#ifndef ofnode_is_available
#define ofnode_is_available(node) ofnode_is_enabled(node)
#endif

#ifndef DISPLAY_FLAGS_SYNC_POSEDGE
#define DISPLAY_FLAGS_SYNC_POSEDGE (1 << 11)
#define DISPLAY_FLAGS_SYNC_NEGEDGE (1 << 12)
#endif

#endif
