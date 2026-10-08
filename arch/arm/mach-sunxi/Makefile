# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
#
# Based on some other Makefile
# (C) Copyright 2000-2003
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.

obj-y	+= board.o
obj-y	+= cpu_info.o
obj-y	+= dram_helpers.o
obj-$(CONFIG_SUN6I_PRCM)	+= prcm.o
obj-$(CONFIG_SUNXI_SYSCON)	+= syscon.o
obj-$(CONFIG_AXP_PMIC_BUS)	+= pmic_bus.o
obj-$(CONFIG_MACH_SUNIV)	+= clock_sun6i.o
obj-$(CONFIG_MACH_SUN4I)	+= clock_sun4i.o
obj-$(CONFIG_MACH_SUN5I)	+= clock_sun4i.o
obj-$(CONFIG_MACH_SUN6I)	+= clock_sun6i.o
obj-$(CONFIG_MACH_SUN7I)	+= clock_sun4i.o
obj-$(CONFIG_MACH_SUN50I)	+= clock_sun6i.o
ifdef CONFIG_MACH_SUN8I_A83T
obj-y				+= clock_sun8i_a83t.o
else
obj-$(CONFIG_MACH_SUN8I)	+= clock_sun6i.o
endif
obj-$(CONFIG_MACH_SUN9I)	+= clock_sun9i.o gtbus_sun9i.o
obj-$(CONFIG_SUN50I_GEN_H6)	+= clock_sun50i_h6.o
obj-$(CONFIG_SUNXI_GEN_NCAT2)	+= clock_sun50i_h6.o
ifndef CONFIG_ARM64
obj-y	+= timer.o
endif

ifdef CONFIG_XPL_BUILD
obj-y				+= clock.o
obj-$(CONFIG_MACH_SUNIV)	+= dram_suniv.o
obj-$(CONFIG_DRAM_SUN4I)	+= dram_sun4i.o
obj-$(CONFIG_DRAM_SUN6I)	+= dram_sun6i.o
obj-$(CONFIG_DRAM_SUN8I_A23)	+= dram_sun8i_a23.o
obj-$(CONFIG_DRAM_SUN8I_A33)	+= dram_sun8i_a33.o
obj-$(CONFIG_DRAM_SUN8I_A83T)	+= dram_sun8i_a83t.o
obj-$(CONFIG_DRAM_SUN9I)	+= dram_sun9i.o
obj-$(CONFIG_SPL_SPI_SUNXI)	+= spl_spi_sunxi.o
obj-$(CONFIG_SUNXI_DRAM_DW)	+= dram_sunxi_dw.o
obj-$(CONFIG_SUNXI_DRAM_DW)	+= dram_timings/
obj-$(CONFIG_DRAM_SUN50I_H6)	+= dram_sun50i_h6.o dram_dw_helpers.o
obj-$(CONFIG_DRAM_SUN50I_H6)	+= dram_timings/
obj-$(CONFIG_DRAM_SUN50I_H616)	+= dram_sun50i_h616.o dram_dw_helpers.o
obj-$(CONFIG_DRAM_SUN50I_H616)	+= dram_timings/
obj-$(CONFIG_DRAM_SUN50I_A133)	+= dram_sun50i_a133.o
obj-$(CONFIG_DRAM_SUN50I_A133)	+= dram_timings/
obj-$(CONFIG_MACH_SUN55I_A523)	+= dram_sun55i_a523.o dram_dw_helpers.o
obj-$(CONFIG_DRAM_SUN55I_A523)	+= dram_timings/
obj-$(CONFIG_MACH_SUN60I_A733)	+= dram_sun60i_a733.o

ifdef CONFIG_MACH_SUN60I_A733
ARC_CROSS_COMPILE ?= arc-linux-gnu-

quiet_cmd_gen_pmu_fw = GENPMU  $@
cmd_gen_pmu_fw = $(PYTHON3) $(srctree)/$(src)/pmu/build_pmu_fw.py \
		--cross-compile $(ARC_CROSS_COMPILE) \
		--src-dir $(srctree)/$(src)/pmu \
		--out $@ $(if $(CONFIG_SUNXI_PMU_DEBUG_TELEMETRY),--debug-telemetry) \
		$(if $(CONFIG_SUNXI_PMU_SINGLE_RANK),--single-rank) \
		$(if $(CONFIG_SUNXI_PMU_FAST_DELAYS),--fast-delays)

$(obj)/dram_sun60i_a733_fw.h: $(src)/pmu/lpddr5_pmu_internal.h \
                              $(src)/pmu/pmu_primitives.c \
                              $(src)/pmu/pmu_clock.c \
                              $(src)/pmu/pmu_telemetry.c \
                              $(src)/pmu/pmu_cbt.c \
                              $(src)/pmu/pmu_deskew.c \
                              $(src)/pmu/pmu_eye.c \
                              $(src)/pmu/pmu_train.c \
                              $(src)/pmu/dwc_ddrphy_pmu_regs.h \
                              $(src)/pmu/sunxi_pmu_abi.h \
                              $(src)/pmu/pmu.ld \
                              $(src)/pmu/build_pmu_fw.py
	$(call cmd,gen_pmu_fw)

$(obj)/dram_sun60i_a733.o: $(obj)/dram_sun60i_a733_fw.h
CFLAGS_dram_sun60i_a733.o += -I$(obj)
endif
endif
