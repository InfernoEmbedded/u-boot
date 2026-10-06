#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""
Build script: Compiles decompiled C training firmware for ARC EM4
and generates the C header for U-Boot.
"""

import argparse
import subprocess
import struct
import os
import shutil
import sys
import tempfile

def run_cmd(cmd, cwd):
    ret = subprocess.run(cmd, shell=True, cwd=cwd, capture_output=True, text=True)
    if ret.returncode != 0:
        sys.stderr.write(f"ERROR: Command failed with exit code {ret.returncode}\n")
        sys.stderr.write(f"Command: {cmd}\n")
        sys.stderr.write(f"STDOUT:\n{ret.stdout}\n")
        sys.stderr.write(f"STDERR:\n{ret.stderr}\n")
        sys.exit(1)
    return ret.stdout

def build_lpddr5_firmware(src_dir, cross_compile, tmp_dir, debug_telemetry=False,
                           single_rank=False, fast_delays=False):
    src_dir = os.path.abspath(src_dir)
    obj_c = os.path.join(tmp_dir, "lpddr5_c.o")
    elf = os.path.join(tmp_dir, "lpddr5.elf")
    bin_file = os.path.join(tmp_dir, "lpddr5.bin")

    c_src = os.path.join(src_dir, "lpddr5_pmu_train.c")
    ld_script = os.path.join(src_dir, "pmu.ld")

    # 1. Compile C training firmware
    cflags = "-mcpu=em4 -mrf16 -mcode-density -mdiv-rem -mbarrel-shifter -mmpy-option=mpy -fno-branch-count-reg -EL -Os -fno-caller-saves -fno-stack-protector -fno-builtin -std=gnu11"
    if debug_telemetry:
        cflags += " -DPMU_DEBUG_TELEMETRY"
    if single_rank:
        cflags += " -DCONFIG_SUNXI_PMU_SINGLE_RANK"
    if fast_delays:
        cflags += " -DCONFIG_SUNXI_PMU_FAST_DELAYS"
    run_cmd(f"{cross_compile}gcc -c {cflags} -I{src_dir} {c_src} -o {obj_c}", src_dir)

    # 2. Link standalone C firmware: .vectors placed at 0x00000000 by pmu.ld
    run_cmd(f"{cross_compile}ld -EL -T {ld_script} {obj_c} -o {elf}", src_dir)

    # 3. Extract flat binary
    run_cmd(f"{cross_compile}objcopy -O binary {elf} {bin_file}", src_dir)

    with open(bin_file, "rb") as f:
        data = f.read()

    return data


def generate_header(lpddr5_bin, out_header):
    def format_array(name, data):
        if len(data) % 2 != 0:
            data += b"\x00"
        halfwords = struct.unpack(f"<{len(data)//2}H", data)
        lines = []
        lines.append(f"#define SUN60I_A733_{name.upper()}_FW_SIZE {len(data)}\n")
        lines.append(f"static const u16 sun60i_a733_{name.lower()}_fw[] = {{\n")
        for i in range(0, len(halfwords), 8):
            chunk = halfwords[i:i+8]
            line = "    " + ", ".join(f"0x{w:04x}" for w in chunk) + ",\n"
            lines.append(line)
        lines.append("};\n\n")
        return "".join(lines)

    content = []
    content.append("// SPDX-License-Identifier: GPL-2.0+\n")
    content.append("/*\n")
    content.append(" * Synopsys DesignWare DDR PHY training firmware for Allwinner A733\n")
    content.append(" * Automatically generated at build time from clean C source:\n")
    content.append(" *   arch/arm/mach-sunxi/pmu/lpddr5_pmu_train.c\n")
    content.append(" */\n\n")
    content.append("#ifndef _DRAM_SUN60I_A733_FW_H\n")
    content.append("#define _DRAM_SUN60I_A733_FW_H\n\n")
    content.append("#include <linux/types.h>\n\n")
    content.append(format_array("LPDDR5", lpddr5_bin))
    content.append("#endif /* _DRAM_SUN60I_A733_FW_H */\n")

    header_text = "".join(content)
    os.makedirs(os.path.dirname(os.path.abspath(out_header)), exist_ok=True)
    with open(out_header, "w") as f:
        f.write(header_text)

def main():
    parser = argparse.ArgumentParser(description="Build Allwinner A733 PMU Firmware Header")
    parser.add_argument("--cross-compile", default="arc-linux-gnu-", help="ARC cross compile prefix")
    parser.add_argument("--src-dir", default=os.path.dirname(os.path.abspath(__file__)), help="Source directory")
    parser.add_argument("--out", required=True, help="Output C header file")
    parser.add_argument("--debug-telemetry", action="store_true", help="Enable PMU telemetry and trace logging")
    parser.add_argument("--single-rank", action="store_true", help="Optimize for single-rank DRAM topology")
    parser.add_argument("--fast-delays", action="store_true", help="Enable aggressive micro-delays and settling")
    args = parser.parse_args()
    arc_gcc = f"{args.cross_compile}gcc"
    has_arc = shutil.which(arc_gcc) is not None
    fallback_header = os.path.join(args.src_dir, "lpddr5_pmu_fw.h")

    if not has_arc:
        if os.path.exists(fallback_header):
            sys.stderr.write(f"NOTE: {arc_gcc} not found on PATH. Using pre-generated {fallback_header}\n")
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            with open(fallback_header, "r") as src, open(args.out, "w") as dst:
                dst.write(src.read())
            return
        sys.stderr.write(f"ERROR: {arc_gcc} not found on PATH and {fallback_header} does not exist!\n")
        sys.exit(1)

    with tempfile.TemporaryDirectory() as tmp_dir:
        lpddr5_bin = build_lpddr5_firmware(args.src_dir, args.cross_compile, tmp_dir,
                                          args.debug_telemetry, args.single_rank, args.fast_delays)
        print(f"LPDDR5 FW Size: {len(lpddr5_bin)} bytes ({65536 - len(lpddr5_bin)} bytes headroom)")
        if len(lpddr5_bin) > 65536:
            sys.stderr.write(f"ERROR: Firmware size {len(lpddr5_bin)} exceeds 65536 bytes limit!\n")
            sys.exit(1)
        generate_header(lpddr5_bin, args.out)

if __name__ == "__main__":
    main()
