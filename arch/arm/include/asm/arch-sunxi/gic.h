/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __SUNXI_GIC_COMPAT_H__
#define __SUNXI_GIC_COMPAT_H__

#include <irq_func.h>

static inline int irq_enable(int irq_no) { return 0; }
static inline int irq_disable(int irq_no) { return 0; }

#endif /* __SUNXI_GIC_COMPAT_H__ */
