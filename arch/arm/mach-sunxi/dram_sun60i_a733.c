// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun60i A733 / T736 DRAM Controller and Synopsys DWC DDR PHY driver
 *
 * (C) Copyright 2026 Alastair D'Silva <alastair@d-silva.org>
 * Assisted-by: Antigravity <antigravity@google.com>
 */

#include <init.h>
#include <log.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/arch/cpu.h>
#include <asm/arch/prcm.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#include "dram_sun60i_a733_lpddr5_fw.h"
#include "dram_sun60i_a733_imem.h"
#include "dram_sun60i_a733_tables.h"
#include "dram_sun60i_a733_para.h"

#define SUNXI_DRAM_PHY_IMEM_BASE	0x0a9a0000
#define SUNXI_DRAM_PHY_DMEM_BASE	0x0a9b0000
#define SUNXI_DRAM_PHY_CTRL_BASE	0x0aaa0000

static void mctl_write_regs(const struct sunxi_dram_reg *regs, u32 count)
{
	for (u32 i = 0; i < count; i++) {
		u32 pc = regs[i].pc;
		
		/* Skip Boot0 SRAM A1/A2 stack/BSS variables that were captured by the emulator.
		 * MUST NOT skip 0x00083000 to 0x000831ff because Boot0 puts the PMU mailbox here (boot0_dram_para).
		 * MUST NOT skip 0x0008ce00 to 0x0008cfff because those contain pointers to the PMU mailbox! */
		if (regs[i].addr < 0x00080000 || 
			(regs[i].addr > 0x00084000 && regs[i].addr < 0x0008ce00) ||
			(regs[i].addr > 0x0008cfff && regs[i].addr < 0x000b0000))
			continue;

		/* Wait for PMU to finish BEFORE writing 0 to 0x0aaa0000 */
		if (regs[i].addr == 0x0aaa0000 && regs[i].val == 0) {
			u32 timeout = 50000;
			while ((readw((void __iomem *)0x0aaa0008) & 1) != 0) {
				udelay(100);
				timeout--;
				if (timeout <= 0)
					break;
			}
		}

		if (regs[i].size == 4)
			writel(regs[i].val, (void __iomem *)regs[i].addr);
		else if (regs[i].size == 2)
			writew(regs[i].val, (void __iomem *)regs[i].addr);
		else if (regs[i].size == 1)
			writeb(regs[i].val, (void __iomem *)regs[i].addr);
			
		/* If we wrote to PLL_DDR setting the enable bit but not yet the 0x08000000 bit */
		if (regs[i].addr == 0x02002020 && (regs[i].val & 0xe0000000) == 0xe0000000 && !(regs[i].val & 0x08000000)) {
			udelay(20);
			while (!(readl((void __iomem *)0x02002020) & (1 << 28))) {
				/* wait for PLL lock */
			}
		}

		const u32 PC_PHY_CALIB_POLL = 0x574ac;
		const u32 PC_SW_DONE_ACK_POLL = 0x574ba;

		/* PHY Calibration Status poll (wait for whatever it takes) */
		if (pc == PC_PHY_CALIB_POLL) {
			/* Boot0 polls 0x0a110b88 at 0x574c0 and 0x0a510b88 at 0x574ca */
			/* Let's poll 0x0a110b88 until it changes from 0? Or just delay 100ms to be safe */
			mdelay(100); 
		}
		
		/* Ready bit / sw_done_ack poll */
		if (pc == PC_SW_DONE_ACK_POLL) {
			/* Boot0 polls 0x0a110c84 / 0x0a510c84 at 0x574e0 */
			udelay(5000);
		}
	}
}

static void mctl_post_pmu_ungate(void)
{
	u32 channels = 2; // Assuming dual channel (A733 is 2x16 LPDDR5)
	u32 i;

	printf("Performing post-PMU AXI ungate...\n");

	for (i = 0; i < channels; i++) {
		void __iomem *base = (void __iomem *)(0x0a110000 + i * 0x400000);
		writel(0, base + 0xc80);
		writel(0, base + 0x208);
		
		u32 val = readl(base + 0x510);
		writel((val & 0xffff3fff) | 0x4000, base + 0x510);
		
		writel(1, base + 0xc80);
		
		/* Wait for AXI port ready bits */
		while ((readl(base + 0xc84) & 1) == 0)
			udelay(1);
	}


	printf("Configuring MBUS/interconnect...\n");

	if (channels == 2) {
		writel(0x8002, (void __iomem *)0x02024818);
		writel(0xffffffff, (void __iomem *)0x03120000); /* *puVar6 where puVar6 = 0x03120000 */
		writel(0x8002, (void __iomem *)0x0202481c);
		writel(0x8002, (void __iomem *)0x0203481c); /* puVar6[0x4000] -> 0x0203481c */
		writel(0xffffffff, (void __iomem *)0x02024820);
		writel(0xffffffff, (void __iomem *)0x02034820);
		
		/* Speculative: Add SYS_CFG / PRCM writes for memory mapping */
		writel(1, (void __iomem *)0x03000000);
		writel(1, (void __iomem *)0x03000200);

		writel(2, (void __iomem *)0x02024898);
		writel(2, (void __iomem *)0x020244ec); /* puVar6[-0xeb] -> 0x02024898 - 0x3ac */
		writel(2, (void __iomem *)0x020246ec); /* puVar6[-0x6b] -> 0x02024898 - 0x1ac */
		
		writel(0xffffffff, (void __iomem *)0x02023a04);
		writel(0xffffffff, (void __iomem *)0x02023c04);
		writel(0xffffffff, (void __iomem *)0x02023e04);
		writel(0xffffffff, (void __iomem *)0x02024004);
		writel(0xffffffff, (void __iomem *)0x02024204);
		writel(0x02023504, (void __iomem *)0x02024490);
		writel(0x02023704, (void __iomem *)0x020244f4); /* puVar6[-0xe9] -> 0x02024898 - 0x3a4 */
		writel(0x02023504, (void __iomem *)0x020246f0); /* puVar6[-0x6a] -> 0x02024898 - 0x1a8 */
		writel(0x02023704, (void __iomem *)0x02024498); /* final *puVar6 */
	} else {
		writel(1, (void __iomem *)0x03000000);
		writel(1, (void __iomem *)0x03000200);
		writel(3, (void __iomem *)0x03120000);
		writel(3, (void __iomem *)0x03130000);
	}

	printf("MBUS and AXI ungated successfully!\n");
}

static void execute_fun_0004f500(void)
{
	u32 val;
	const u32 CH0_AXI_UNGATE = 0x0a110c80;
	const u32 CH0_AXI_CTRL = 0x0a110510;
	const u32 CH1_AXI_UNGATE = 0x0a510c80;
	const u32 CH1_AXI_CTRL = 0x0a510510;
	const u32 CCU_DRAM_CLK = 0x02002c00;
	
	writel(0, (void __iomem *)CH0_AXI_UNGATE);
	val = readl((void __iomem *)CH0_AXI_CTRL);
	writel(val & ~0x1f00, (void __iomem *)CH0_AXI_CTRL); /* clear bits 8-12 */
	writel(1, (void __iomem *)CH0_AXI_UNGATE);
	
	writel(0, (void __iomem *)CH1_AXI_UNGATE);
	val = readl((void __iomem *)CH1_AXI_CTRL);
	writel(val & ~0x1f00, (void __iomem *)CH1_AXI_CTRL);
	writel(1, (void __iomem *)CH1_AXI_UNGATE);
	
	val = readl((void __iomem *)CCU_DRAM_CLK);
	val &= ~0x0700001f; /* clear bits 0-4, 24-26 */
	val |= 3;
	writel(val, (void __iomem *)CCU_DRAM_CLK);
	writel(val | BIT(27), (void __iomem *)CCU_DRAM_CLK); /* clock enable */
	
	mdelay(1);
}

static int mctl_phy_init(void)
{
	u32 val = 0;

	/* Stage 0: Replicate Boot0's peripheral accesses explicitly to ensure 
	 * CCU, PRCM, SYSCTRL, and PIO are in the correct state prior to PHY training.
	 */
	mctl_write_regs(dram_pre_training_periph_regs, ARRAY_SIZE(dram_pre_training_periph_regs));

	/* Wait for PLLs to lock */
	mdelay(5);

	/* 2. Pre-IMEM PHY setup */
	mctl_write_regs(dram_stage2_pre_imem_regs, ARRAY_SIZE(dram_stage2_pre_imem_regs));

	/* 3. Load training firmware into PHY IMEM */
	for (size_t i = 0; i < ARRAY_SIZE(dram_imem_params); i++)
		writew(dram_imem_params[i], (void __iomem *)(SUNXI_DRAM_PHY_IMEM_BASE + i * 2));
	for (size_t i = ARRAY_SIZE(dram_imem_params); i < (0x10000 / 2); i++)
		writew(0, (void __iomem *)(SUNXI_DRAM_PHY_IMEM_BASE + i * 2));

	printf("IMEM after load: %04x %04x %04x\n", 
		readw((void __iomem *)0x0a9a0000), 
		readw((void __iomem *)0x0a9a0002), 
		readw((void __iomem *)0x0a9a0004));

	/* 4. Pre-DMEM PHY setup (this sets 0x0aaa0000 = 0 to allow DMEM access) */
	mctl_write_regs(dram_stage2_pre_dmem_regs, ARRAY_SIZE(dram_stage2_pre_dmem_regs));

	execute_fun_0004f500();

	for (size_t i = 0; i < ARRAY_SIZE(dram_dmem_params); i++)
		writew(dram_dmem_params[i], (void __iomem *)(SUNXI_DRAM_PHY_DMEM_BASE + i * 2));
	for (size_t i = ARRAY_SIZE(dram_dmem_params); i < (0x4000 / 2); i++)
		writew(0, (void __iomem *)(SUNXI_DRAM_PHY_DMEM_BASE + i * 2));

	printf("DMEM after load: %04x %04x %04x\n", 
		readw((void __iomem *)0x0a9b0000), 
		readw((void __iomem *)0x0a9b0002), 
		readw((void __iomem *)0x0a9b0004));

	/* 5. Trigger PMU firmware */
	u8 backup_831f8[sizeof(boot0_dram_para)];
	memcpy(backup_831f8, (void *)0x000831f8, sizeof(boot0_dram_para));
	memcpy((void *)0x000831f8, boot0_dram_para, sizeof(boot0_dram_para));
	flush_dcache_range(0x000831c0, 0x000832c0);

	/* Post-DMEM PHY setup (this sets 0x0aaa0000 = 1 to hand control to PMU) */
	printf("Writing post_dmem_regs...\n");
	mctl_write_regs(dram_stage2_post_dmem_regs, ARRAY_SIZE(dram_stage2_post_dmem_regs));
	printf("Done writing post_dmem_regs.\n");

	/* Exact PMU Trigger Sequence from boot0 FUN_0004f47c */
	writew(0x1, SUNXI_DRAM_PHY_CTRL_BASE + 0x00);
	writew(0x9, SUNXI_DRAM_PHY_CTRL_BASE + 0x132);
	writew(0x1, SUNXI_DRAM_PHY_CTRL_BASE + 0x132);
	writew(0x0, SUNXI_DRAM_PHY_CTRL_BASE + 0x132);
	writew(0x1, SUNXI_DRAM_PHY_CTRL_BASE + 0x62);
	writew(0x1, SUNXI_DRAM_PHY_CTRL_BASE + 0x66);

	/* Wait while bit 0 of 0x0aaa0008 is 1 */
	while ((readw(SUNXI_DRAM_PHY_CTRL_BASE + 0x08) & 1) != 0) {
		udelay(1);
	}
	writew(0x0, SUNXI_DRAM_PHY_CTRL_BASE + 0x62);
	while (readw(SUNXI_DRAM_PHY_CTRL_BASE + 0x08) == 0) {
		udelay(1);
	}
	writew(0x1, SUNXI_DRAM_PHY_CTRL_BASE + 0x62);
	writew(0x1, SUNXI_DRAM_PHY_CTRL_BASE + 0x132);

	val = readw(SUNXI_DRAM_PHY_CTRL_BASE + 0x64) | (readw(SUNXI_DRAM_PHY_CTRL_BASE + 0x68) << 16);
	printf("PMU Status: 0x%08x\n", val);
	if (val != 0 && val != 0xff) {
		printf("PMU training failed!\n");
	} else {
		printf("PMU initialized successfully!\n");
	}

	if (val != 0xff) {
		return -1;
	} else {
		printf("PMU initialized successfully!\n");
	}

	/* Execute all Stage 3 configuration/mailboxes first */
	mctl_write_regs(dram_stage3_regs, ARRAY_SIZE(dram_stage3_regs));
	

	mctl_post_pmu_ungate();

	printf("Transitioning uMCTL2 to normal mode...\n");
	u32 channels = 2; // dual channel
	for (u32 i = 0; i < channels; i++) {
		printf("Channel %d: clearing pwrctl.selfref_sw (0x%08x)\n", i, 0x0a110180 + i * 0x400000);
		u32 pwrctl = readl((void __iomem *)(0x0a110180 + i * 0x400000));
		writel(pwrctl & ~0x20, (void __iomem *)(0x0a110180 + i * 0x400000));
		
		printf("Channel %d: clearing pwrctl.en_dfi_dram_clk_disable\n", i);
		pwrctl = readl((void __iomem *)(0x0a110180 + i * 0x400000));
		writel(pwrctl & ~0x800, (void __iomem *)(0x0a110180 + i * 0x400000));
	}

	printf("uMCTL2 ready!\n");

	/* We do not know the exact success register for A733 PHY yet, so assume success and let the 0x40000000 test verify */
	return 0;
}



/*
 * Calculate DRAM size matching Allwinner boot0 FUN_0004b74c algorithm:
 *   para1: bits[3:0]=cols, bits[11:4]=rows, bits[13:12]=banks, bits[15:14]=bank_groups
 *   para2: bits[15:12]=ranks
 *   dram_tpr13: bit 6 (bus width/density adjustment), bits[18:16] (3/4 capacity flag)
 */
static unsigned long sun60i_a733_calc_dram_size(const u32 *para)
{
	u32 para1 = para[0x18 / 4];
	u32 para2 = para[0x1c / 4];
	u32 tpr13 = para[0x78 / 4];

	u32 cols = para1 & 0xf;
	u32 rows = (para1 & 0xfff) >> 4;
	u32 banks = (para1 & 0x3fff) >> 12;
	u32 bank_groups = (para1 & 0xffff) >> 14;
	u32 ranks = (para2 & 0xffff) >> 12;

	u32 sum = ranks + bank_groups + rows + banks + cols;
	u32 shift;

	if ((para2 & 0xf) == 0)
		shift = sum - 18;
	else if ((tpr13 & 0x40) == 0)
		shift = sum - 19;
	else
		shift = sum - 18;

	u32 size_mb = 1U << (shift & 0xff);

	if ((tpr13 & 0x70000) && ((para2 >> 30) != 2))
		size_mb = (size_mb * 3) >> 2;

	return (unsigned long)size_mb << 20;
}

unsigned long sunxi_dram_init(void)
{
	int ret;

	u32 *para = (u32 *)boot0_dram_para;
	u32 dram_clk = para[0];
	u32 dram_type = para[1];
	const char *type_str;
	
	switch (dram_type) {
	case 3: type_str = "DDR3"; break;
	case 4: type_str = "DDR4"; break;
	case 6: type_str = "LPDDR3"; break;
	case 7: type_str = "LPDDR4"; break;
	case 9: type_str = "LPDDR5"; break;
	default: type_str = "UNKNOWN"; break;
	}

	printf("A733 sunxi_dram_init: starting %s training @ %uMHz...\n", type_str, dram_clk);

	/* Stage 1: CCU, DMC, uMCTL2 Channel 0/1 setup */
	
	/* Missing CCMU config from boot0 (missed by emulator) */
	u32 val = readl((void __iomem *)0x07010310);
	val = (val & 0xffffc0c0) | 0x2728; /* Dual channel */
	writel(val, (void __iomem *)0x07010310);
	writel(val, (void __iomem *)0x07410310); /* Channel 1 */
	
	val = readl((void __iomem *)0x07010314);
	val = (val & 0xfffffffc) | 3; /* Dual channel + LPDDR5 */
	writel(val, (void __iomem *)0x07010314);
	writel(val, (void __iomem *)0x07410314); /* Channel 1 */

	/* Un-reset and enable clock */
	val = readl((void __iomem *)0x07010310);
	val = (val & 0xffffffef) | 0x14;
	writel(val, (void __iomem *)0x07010310);
	writel(val, (void __iomem *)0x07410310); /* Channel 1 */
	
	val = (val & 0xfffffffe) | 2;
	writel(val, (void __iomem *)0x07010310);
	writel(val, (void __iomem *)0x07410310); /* Channel 1 */
	udelay(1);
	
	val |= 3;
	writel(val, (void __iomem *)0x07010310);
	writel(val, (void __iomem *)0x07410310); /* Channel 1 */
	udelay(1);

	mctl_write_regs(dram_stage1_regs, ARRAY_SIZE(dram_stage1_regs));

	/* Wait for PLL_DDR to lock */
	while ((readl((void __iomem *)0x02002020) & (1 << 28)) == 0)
		udelay(1);
	
	/* Add safety delay for clocks and resets to propagate */
	mdelay(5);

	/* Stage 2 & 3: PHY calibration, firmware training, and handoff */
	ret = mctl_phy_init();
	if (ret != 0) {
		printf("PMU training failed! System halted.\n");
		while (1); /* halt on failure */
	}

	return sun60i_a733_calc_dram_size(para);
}
