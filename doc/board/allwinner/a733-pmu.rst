.. SPDX-License-Identifier: GPL-2.0+
.. Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>

Allwinner A733 LPDDR5 PHY Management Unit (PMU)
===============================================

Introduction
------------
The Allwinner A733 (sun60iw2) SoC introduces a redesigned memory subsystem
differing substantially from earlier sunxi SoCs (such as the H6, H616, and A523).
While earlier generations performed DRAM delay line calibration directly on the
main ARM core, the A733 pairs a Synopsys DesignWare uMCTL2 controller with a
Synopsys DDR PHY containing an integrated 32-bit ARC EM4 microcontroller,
termed the PHY Management Unit (PMU).

In proprietary vendor firmware (boot0), DRAM training was handled by an opaque
binary firmware blob loaded into the PMU. For U-Boot, this has been replaced
by a 100% open-source, freestanding C training implementation
(``arch/arm/mach-sunxi/pmu/``) accompanied by a pre-generated
fallback header, ensuring fully transparent and reproducible builds.

Background & Theory of Operation
--------------------------------

Why High-Speed LPDDR5 Requires Training
```````````````````````````````````````
Modern LPDDR5 memory operates at blistering speeds—in our board's case, an
1800 MHz clock yielding 3600 MT/s (million transfers per second). At these
frequencies, each individual data bit (DQ) is valid for only ~277 picoseconds
(the "data eye").

Because copper PCB traces have slightly different physical lengths, and silicon
transistors inside the SoC and DRAM vary with manufacturing process, operating
voltage, and temperature (PVT), signals arrive at slightly different times.
If the memory controller sampled data with fixed delays, electrical noise and
clock skew would corrupt reads and writes instantly.

DRAM training electronically calibrates:

* **Timing Deskew**: Adjusting per-bit programmable Local Clock Delay Lines
  (LCDLs) so all bits (DQ0..DQ7, DM) and byte strobes (DQS/DQSN) arrive at the
  exact same fraction of a nanosecond.
* **Vref Voltage Centering**: Adjusting internal DAC reference voltages to place
  the logic 0/1 decision threshold exactly in the vertical center of the eye.
* **Command/Address (CA) Training**: Synchronizing command lines with the memory
  clock via Command Bus Training (CBT).
* **ZQ Impedance Calibration**: Matching output driver (Ron) and on-die termination
  (ODT) impedance against an external 240-ohm precision calibration resistor to
  prevent transmission line reflections.

Role of the Integrated ARC PMU Core
```````````````````````````````````
Performing millions of microsecond-precision analog delay line measurements
using the main host CPU (ARM Cortex-A76 / A55) would require millions of slow
bus round-trips and complex OS scaffolding.

To solve this, Synopsys integrated a dedicated, self-contained 32-bit RISC
microcontroller core—an ARC EM4—directly inside the PHY silicon. This core is
called the PHY Management Unit (PMU). The PMU has direct, cycle-accurate access
to the analog delay lines and PHY registers, allowing it to sweep timing
envelopes, evaluate BIST test patterns, and lock in optimal values autonomously.

Hardware Architecture
---------------------
The memory subsystem consists of:

* **Synopsys DesignWare uMCTL2**: Dual-channel memory controller managing queuing,
  scheduling, refresh, low-power states, and AXI port arbitration.
* **Synopsys DesignWare DDR PHY**: High-speed physical layer interface supporting
  LPDDR5 (up to 5500 MT/s) and LPDDR4X across two independent 16-bit channels.
* **ARC EM4 PMU Core**: Embedded 32-bit RISC core with Harvard architecture:
  - **IMEM (Instruction Memory / ICCM)**: 64 KiB tightly coupled SRAM.
  - **DMEM (Data Memory / DCCM)**: 64 KiB tightly coupled SRAM.
  - **APB Master & Slave Interfaces**: The PMU masters PHY internal registers to
    sweep analog delay lines, adjust receiver Vref DACs, and measure data eyes.

PHY Silicon Architecture & Slices
`````````````````````````````````
Inside the Synopsys DesignWare DDR PHY Type 9 silicon on Allwinner A733:

* **DBYTE Slices (Data Byte Slices 0..3)**:
  Each DBYTE slice manages the physical analog signals for one 8-bit byte lane
  (DQ0..DQ7, Data Mask DM, and differential Data Strobe DQS/DQSN). Because the
  A733 uses dual 16-bit channels (Ch0 = 16-bit, Ch1 = 16-bit), there are 4 DBYTE
  slices in total:
  - **Slice 0**: Channel 0 Byte Lane 0 (DQ0..DQ7)
  - **Slice 1**: Channel 0 Byte Lane 1 (DQ8..DQ15)
  - **Slice 2**: Channel 1 Byte Lane 0 (DQ0..DQ7)
  - **Slice 3**: Channel 1 Byte Lane 1 (DQ8..DQ15)

* **AC Slices (Address/Command Slices 0..1)**:
  Manage the clock (CK/CKN) and Command/Address (CA0..CA6) signals sent to the
  LPDDR5 memory chips for each channel.

* **PUB (PHY Utility Block)**:
  The central control core containing master PLL configuration, clock gating,
  PMU execution mailboxes, analog bias generators, and host CPU APB interface.

* **LCDL (Local Clock Delay Line)**:
  Analog tapped delay lines consisting of coarse stages and fine vernier stages.
  Adjusting LCDL delay taps shifts signal transitions by picoseconds, enabling
  the PMU to place sampling clock edges precisely in the center of the data eye.

* **CBT (Command Bus Training)**:
  A JEDEC-standard training mode where the memory controller transmits command
  patterns to the DRAM, and the DRAM returns feedback on the DQ lines. This allows
  the PHY to align the CA signals to the memory clock with picosecond accuracy.

* **ZQ Calibration**:
  Calibrates transmitter output impedance (Ron) and receiver on-die termination
  (ODT) against an external precision resistor to prevent transmission line
  signal reflections.

Allwinner A733 Memory Subsystem Register Map
````````````````````````````````````````````
The Allwinner A733 User Manual documents the memory subsystem architecture
under Section 3.2 (System Memory Map), Section 4.1 (CCU), Section 4.2 (PRCM),
Section 5.1 (CPU Subsystem Control), and Section 11.1 (SDRAM Controller DRAMC):

===========================  ==================  ========================================
Peripheral Module            Host ARM Physical   Description / Ref Manual Section
===========================  ==================  ========================================
MEMC0 Subsystem Base         0x0a000000          17 MB Memory Subsystem Window (Sec 3.2)
SMC0 / SMC1                  0x0a000000/010000   Security Memory Controllers (64 KiB each)
DRAMC Common Base            0x0a020000          64 KiB DRAMC Common registers (Sec 3.2)
  DRAMC MAER Reg 0           0x0a021800          Memory Access Enable / Firewall 0
  DRAMC MAER Reg 1           0x0a021808          Memory Access Enable / Firewall 1
CPU AXI2HIF0 / AXI2HIF1      0x0a030000/040000   CPU AXI to Host Interface channels
DRAM Controller 0 (uMCTL2)   0x0a100000          4 MB Channel 0 uMCTL2 Window (Sec 3.2)
  uMCTL2 Ch0 CSR Base        0x0a110000          Synopsys uMCTL2 Core Ch0 registers
DRAM Controller 1 (uMCTL2)   0x0a500000          4 MB Channel 1 uMCTL2 Window (Sec 3.2)
  uMCTL2 Ch1 CSR Base        0x0a510000          Synopsys uMCTL2 Core Ch1 registers
DRAM PHY Base                0x0a900000          8 MB Synopsys DDR PHY Window (Sec 3.2)
  PMU Instruction Memory     0x0a9a0000          64 KiB PMU ICCM (Instruction Memory)
  PMU Data Memory            0x0a9b0000          64 KiB PMU DCCM (Data Memory)
  PMU CSRs / APB Slave       0x0aaa0000          64 KiB PHY Control & Status Registers
===========================  ==================  ========================================

Related System Controllers:

* **CCU Base (0x02002000, Sec 4.1.4)**:
  - ``PLL_DDR_CTRL_REG`` (``0x02002020`` / CCU + ``0x0020``): DRAM Subsystem PLL control.
  - ``MBUS_CLK_REG`` (``0x02002588`` / CCU + ``0x0588``): MBUS clock register.
  - ``DRAM0_CLK_REG`` (``0x02002c00`` / CCU + ``0x0c00``): DRAM0 clock register.
  - ``DRAM0_BGR_REG`` (``0x02002c0c`` / CCU + ``0x0c0c``): DRAM0 bus gating reset.
* **CPU_SUBSYS_CTRL Base (0x08000000, Sec 5.1.5.1)**:
  - ``CPU_DA_DDR_CTRL_REG`` (``0x08000200`` / CPU + ``0x0200``): CPU Direct Access DDR Mux
    (Bit 0: ``0`` = NSI Interconnect, ``1`` = AXI2HIF).
* **STBY_PRCM Base (0x07010000, Sec 4.2.4)**:
  - ``VCC_DRAM_ISO_REG`` (``0x07010250`` / PRCM + ``0x0250``): DRAM Power Isolation (Bit 0 = un-isolate).
  - ``RTC_BGR_REG`` (``0x0701020c`` / PRCM + ``0x020c``): RTC Bus Gating Reset register.
* **RTC Base (0x07090000, Sec 4.2.5)**:
  - ``RTC_GP_DATA_REG0`` (``0x07090100`` / RTC + ``0x0100``): General Purpose Retention Data Reg 0.

Address Mapping from Host ARM Core
``````````````````````````````````
The host ARM CPU accesses the DDR PHY and PMU memories over an internal APB
crossbar:

=======================  ==================  ========================================
Address Range (ARM)      Internal (ARC EM4)  Description
=======================  ==================  ========================================
0x0a920000 - 0x0a927fff  0x90020000          DBYTE Slices 0..3 (4 slices, 8 KiB stride)
0x0a940000 - 0x0a941fff  0x90040000          Common PHY Control & Status
0x0a960000 - 0x0a963fff  0x90060000          AC Slices 0..1 (2 slices, 8 KiB stride)
0x0a982000 - 0x0a983fff  0x90082000          PHY Shadow RAM
0x0a9a0000 - 0x0a9affff  0x00000000          PMU Instruction Memory (IMEM, 64 KiB ICCM)
0x0a9b0000 - 0x0a9bffff  0x80000000          PMU Data Memory (DMEM, 64 KiB DCCM)
0x0a9e0000 - 0x0a9effff  0x900e0000          Address/Command (AC) Slice Base
0x0aa80000 - 0x0aa8ffff  0x90180000          PHY Utility Block (PUB) Registers
0x0aaa0000 - 0x0aaaffff  --                  PMU Control & Mailbox (APB Slave Interface)
=======================  ==================  ========================================

.. note::
   The PHY APB slave interface is strictly **16-bit wide**. All host accesses
   must use 16-bit readw / writew operations. 32-bit accesses (readl / writel)
   cause bus faults or corrupted register writes.

Key PHY Control and Mailbox Registers (relative to ``0x0aaa0000``):

* **0x0000 (PMU_REG_APB_MUX)**: APB bus multiplexer. Writing ``0x0`` grants APB
  access to the host ARM CPU; writing ``0x1`` grants APB access to the ARC PMU.
* **0x0008 (PMU_REG_POLL_STAT)**: Busy poll status. Bit 0 is ``1`` while PMU
  training is in progress, and clears to ``0`` when training completes or idles.
* **0x0062 (PMU_REG_HANDOFF_1)**: Handshake handoff 1. Toggled to trigger execution
  (``0x1``) and acknowledge completion (``0x0`` -> ``0x1``).
* **0x0064 (PMU_REG_STATUS_LO)**: Low 16 bits of PMU return status.
* **0x0066 (PMU_REG_MAILBOX_INT)**: Mailbox interrupt pulse / execution trigger.
* **0x0068 (PMU_REG_STATUS_HI)**: High 16 bits of PMU return status.
* **0x0132 (PMU_REG_RESET)**: PMU microsequencer reset and clock gating control.
  Writing sequence ``0x9`` -> ``0x1`` -> ``0x0`` releases reset and starts execution.

Inter-Processor Communication (IPC ABI)
---------------------------------------
Communication between U-Boot SPL (running on the host ARM core) and the PMU
(running on the ARC EM4 core) is coordinated through a shared mailbox structure
placed at the base of DMEM (offset ``0x0000``, host ``0x0a9b0000`` / ARC ``0x80000000``),
defined in ``arch/arm/mach-sunxi/pmu/sunxi_pmu_abi.h``:

Shared Mailbox Layout (struct pmu_message_block)
```````````````````````````````````````````````

===================  ======  ========================================================
Field                Offset  Description
===================  ======  ========================================================
``dram_type``        0x00    Memory type (``3`` = LPDDR5, ``2`` = LPDDR4)
``cfg_flags``        0x01    Hardware configuration flags
``pstate_flags``     0x04    Power-state flags and channel offset
``dram_freq_mhz``    0x06    DRAM data rate in MT/s (e.g., 3600 for 1800 MHz clock)
``pll_ratio``        0x08    PLL multiplication ratio
``dfi_freq_ratio``   0x0a    DFI frequency divider ratio
``assert_code``      0x0c    PMU assertion diagnostic error code (read on failure)
``sequence_ctrl``    0x10    Training stage bitmask (see below)
``post_threshold``   0x12    Telemetry log filter threshold (``0xff`` = disabled)
``train_mode``       0x14    Training mode / calibration state flags
===================  ======  ========================================================

Training Modes & SequenceCtrl Flags
```````````````````````````````````
The ``sequence_ctrl`` bitfield at DMEM offset ``0x10`` selects which training
and calibration stages the PMU microsequencer executes:

* **PMU_SEQ_FAST_BOOT (0x1001)**:
  Combines ``PMU_SEQ_DEV_INIT`` (Bit 0) and ``PMU_SEQ_LPCA_INIT`` (Bit 12).
  Initializes PHY PLLs, pad drivers, and Command/Address training in ~6.6 ms.
  SPL then restores cached timing registers, avoiding the full multi-second sweep.
* **PMU_SEQ_FULL_TRAIN (0x125f)**:
  Executes the complete calibration pipeline (~2.42s):
  - **Bit 0 (DEV_INIT)**: Device init, PLL lock, Command Bus Training (CBT) entry, ZQ cal.
  - **Bit 1 (STAGE_VREF)**: Receiver Vref voltage centering.
  - **Bit 2 (STAGE_DCD)**: Duty cycle distortion (DCD) correction.
  - **Bit 3 (STAGE_DQS)**: Read/Write DQS strobe timing centering.
  - **Bit 4 (BIST_SEARCH_WIN)**: BIST test pattern search window setup.
  - **Bit 6 (SEARCH_WIN)**: 2D eye margin search (delay taps vs Vref DAC).
  - **Bit 7 (ZQ_CAL)**: ZQ pad driver output impedance (Ron) and ODT tuning.
  - **Bit 8 (DESKEW_SWEEP)**: Per-bit DQ deskew delay line tuning across all byte lanes.
  - **Bit 9 (MARGIN_STEP)**: Timing margin envelope evaluation.
  - **Bit 10 (LANE_WIN_SWEEP)**: Multi-lane window sweep across all 4 byte lanes.
  - **Bit 12 (LPCA_INIT)**: Low-Power Command/Address (LPCA) training.

Execution & Handshake Sequence
``````````````````````````````
1. **Firmware Loading**:
   SPL grants APB ownership to host (``writew(0x0, 0x0aaa0000)``) and copies
   the compiled PMU binary into IMEM (``0x0a9a0000``).
2. **Parameter Configuration**:
   SPL populates ``struct pmu_message_block`` at DMEM base (``0x0a9b0000``),
   streams the non-zero PHY parameter tables (``dram_dmem_blocks``), and zeroes
   the remaining DMEM space.
3. **Execution Trigger**:
   - SPL grants APB ownership to PMU: ``writew(0x1, 0x0aaa0000)``.
   - SPL cycles PMU reset: ``writew(0x9, 0x0aaa0132)`` -> ``writew(0x1, 0x0aaa0132)`` -> ``writew(0x0, 0x0aaa0132)``.
   - SPL pulses execution triggers: ``writew(0x1, 0x0aaa0062)`` (HANDOFF_1) and
     ``writew(0x1, 0x0aaa0066)`` (MAILBOX_INT).
4. **PMU Training Execution**:
   The ARC EM4 core executes the stages selected by ``sequence_ctrl``, writing
   calibrated delay and Vref settings directly into the PHY registers.
5. **Completion Handshake**:
   - The ARC core signals completion by clearing bit 0 of ``PMU_REG_POLL_STAT``
     and writing status to ``PMU_REG_STATUS_LO`` / ``PMU_REG_STATUS_HI``.
   - SPL polls ``PMU_REG_POLL_STAT`` (offset ``0x0008``) until bit 0 is 0.
   - SPL reads the 32-bit status:
     - ``0x00000007`` (**PMU_STATUS_SUCCESS**): Training passed.
     - ``0x000000ff`` (**PMU_STATUS_FATAL_ERROR**): Training aborted on assertion.
   - SPL completes handshake: writes ``0x0`` to ``PMU_REG_HANDOFF_1``, waits for ack,
     then writes ``0x1`` to ``PMU_REG_HANDOFF_1`` and ``PMU_REG_RESET``.
   - SPL reclaims APB bus ownership: ``writew(0x0, 0x0aaa0000)``.

Two-Tier DRAM Timing Cache
--------------------------
Full PMU matrix calibration sweeps 128 delay taps across all data bytes and
channels, requiring approximately 2.42 seconds. To achieve sub-10ms fast boot
while retaining optimal memory timing stability, U-Boot SPL implements a
two-tier timing cache:

* **Tier 1 (Retention SRAM at 0x0008f000)**:
  SRAM retention banks remain powered during warm resets and low-power standby.
  SPL validates the cache header magic (``0x4452414d``), version, clock frequency,
  and CRC32. If valid, the PMU is invoked in fast-boot mode (``PMU_SEQ_FAST_BOOT``,
  ~6.6 ms) for device/LPCA init, and the cached trained PHY registers are restored.
* **Tier 2 (SPI NOR Flash at offset 0x003f0000 / Sector 1008)**:
  On cold power-on, SRAM retention is lost. SPL probes SPI NOR flash at offset
  ``0x003f0000`` (4032 KiB). If a valid cache image matching the current board
  configuration is found, timings are restored in ~8 ms total.
* **Fallback & Cache Population**:
  If both caches are invalid (first boot or flash erase), SPL triggers a full
  PMU calibration sweep (``PMU_SEQ_FULL_TRAIN``). Upon completion, SPL snapshots
  the trained PHY registers and commits the resulting cache image to both
  Retention SRAM and SPI NOR flash, ensuring subsequent boots use Tier 1 or Tier 2.

Diagnostics & Telemetry
-----------------------
* **Assertion Failure Code**:
  If training fails with status ``0x000000ff``, the PMU firmware logs the exact
  stage failure code at DMEM offset ``0x0c`` (``assert_code``). SPL dumps this
  code along with diagnostic PHY status registers.
* **Stage Profiling Telemetry**:
  When ``CONFIG_SUNXI_PMU_DEBUG_TELEMETRY`` is enabled, the PMU firmware records
  hardware cycle timestamps at each stage transition into DMEM offsets
  ``0x0300 - 0x0330``. SPL formats and displays these timestamps to profile stage
  execution durations.

Build System & Toolchain Integration
------------------------------------
The PMU firmware source is located at ``arch/arm/mach-sunxi/pmu/``:

* ``pmu_primitives.c``: Low-level I/O, reset vector, hardware delays, and runtime primitives.
* ``pmu_clock.c``: PHY PLL control, clock gating, ratio latching, and frequency math.
* ``pmu_telemetry.c``: Host mailbox IPC, stage profiling, and diagnostic trace dumps.
* ``pmu_cbt.c``: Command-Bus Training (CBT) pulse sequences and calibration tables.
* ``pmu_deskew.c``: DBYTE & AC slice delay sweeps, LCDL line tuning, and BIST routines.
* ``pmu_eye.c``: 2D eye diagram margin searching, boundary detection, and centroid alignment.
* ``pmu_train.c``: Firmware entry, training stage orchestration, and execution dispatcher.
* ``lpddr5_pmu_internal.h``: Internal function prototypes and MMIO inline accessors.
* ``dwc_ddrphy_pmu_regs.h``: Synopsys DDR PHY register and memory definitions.
* ``sunxi_pmu_abi.h``: Inter-processor communication ABI and mailbox definitions.
* ``pmu.ld``: ARC linker script mapping vectors to 0x00000000.
* ``build_pmu_fw.py``: Firmware compilation and C array packaging script.
* ``lpddr5_pmu_fw.h``: Pre-generated C array fallback header.

Toolchain Independence
``````````````````````
Building U-Boot does **not** strictly require an ARC toolchain:

* If ``arc-linux-gnu-gcc`` is present on the host ``PATH``, the build system
  re-compiles the PMU C source files into binary form and regenerates the header
  whenever the source is modified.
* If ``arc-linux-gnu-gcc`` is **not** present, ``build_pmu_fw.py`` transparently
  uses the checked-in fallback header ``lpddr5_pmu_fw.h``, ensuring standard
  ARM64 cross-compilation environments build without extra dependencies.
