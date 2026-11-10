#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""
Single-pass ELF builder for Allwinner A733 LPDDR5 ARC EM4 PMU Firmware.
"""

import argparse
import glob
import os
import shutil
import struct
import subprocess
import sys
import tempfile


def resolve_arc_toolchain(preferred_prefix):
    """Locate an ARC GCC toolchain prefix across PATH and U-Boot CI locations."""
    candidates = [
        preferred_prefix,
        "arc-linux-gnu-",
        "arc-linux-",
        "arc-elf32-",
    ]
    for pattern in (
        "/opt/gcc-*-nolibc/arc-linux/bin/arc-linux-",
        os.path.expanduser("~/.buildman-toolchains/gcc-*-nolibc/arc-linux/bin/arc-linux-"),
    ):
        candidates.extend(sorted(glob.glob(pattern), reverse=True))

    for prefix in candidates:
        if shutil.which(f"{prefix}gcc") and shutil.which(f"{prefix}objcopy"):
            return prefix

    raise RuntimeError(
        f"ARC toolchain not found (tried '{preferred_prefix}gcc' and standard "
        "PATH / buildman / /opt locations)"
    )


def main():
    pmu_dir_default = os.path.abspath(os.path.dirname(__file__))
    repo_root = os.path.abspath(os.path.join(pmu_dir_default, "..", "..", "..", ".."))
    default_out = os.path.join(repo_root, "arch/arm/mach-sunxi/dram_sun60i_a733_pmu_fw.h")

    parser = argparse.ArgumentParser(
        description="Build Allwinner A733 LPDDR5 ARC EM4 PMU firmware."
    )
    parser.add_argument("--cross-compile", default="arc-linux-gnu-", help="ARC toolchain prefix")
    parser.add_argument("--src-dir", default=pmu_dir_default, help="PMU firmware source directory")
    parser.add_argument("--out", default=default_out, help="Output C header path")
    args = parser.parse_args()

    src_dir = os.path.abspath(args.src_dir)
    ld_script = os.path.join(src_dir, "pmu.ld")
    output_h_path = os.path.abspath(args.out)
    prefix = resolve_arc_toolchain(args.cross_compile)
    gcc = f"{prefix}gcc"
    objcopy = f"{prefix}objcopy"

    c_files = sorted(glob.glob(os.path.join(src_dir, "*.c")))
    if not c_files:
        raise RuntimeError(f"No C source files found in {src_dir}")

    cflags = [
        "-mcpu=em4",
        "-mrf16",
        "-mcode-density",
        "-mbranch-index",
        "-mcode-density-frame",
        "-mdiv-rem",
        "-mbarrel-shifter",
        "-mmpy-option=mpy",
        "-fno-branch-count-reg",
        "-EL",
        "-Os",
        "-fno-caller-saves",
        "-fno-ipa-ra",
        "-ffixed-r26",
        "-ffixed-r27",
        "-fno-stack-protector",
        "-fno-builtin",
        "-ffunction-sections",
        "-fdata-sections",
        "-std=gnu11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{src_dir}",
    ]

    with tempfile.TemporaryDirectory(prefix="pmu_fw_") as tmpdir:
        objs = []
        for src_path in c_files:
            base = os.path.splitext(os.path.basename(src_path))[0]
            obj_path = os.path.join(tmpdir, f"{base}.o")
            subprocess.check_call([gcc, *cflags, "-c", src_path, "-o", obj_path])
            objs.append(obj_path)

        elf_path = os.path.join(tmpdir, "pmu.elf")
        bin_path = os.path.join(tmpdir, "pmu.bin")
        ld_cmd = [
            gcc,
            "-mcpu=em4",
            "-mrf16",
            "-mcode-density",
            "-mdiv-rem",
            "-mbarrel-shifter",
            "-mmpy-option=mpy",
            "-EL",
            "-Os",
            "-nostdlib",
            f"-Wl,-L,{src_dir}",
            f"-Wl,-T,{ld_script}",
            "-Wl,--gc-sections",
            "-Wl,--build-id=none",
            *objs,
            "-o",
            elf_path,
        ]
        subprocess.check_call(ld_cmd)
        subprocess.check_call([objcopy, "-O", "binary", elf_path, bin_path])

        with open(bin_path, "rb") as bf:
            raw = bf.read()

    if len(raw) % 2 != 0:
        raw += b"\x00"

    hwords = list(struct.unpack(f"<{len(raw) // 2}H", raw))
    while hwords and hwords[-1] == 0:
        hwords.pop()

    os.makedirs(os.path.dirname(output_h_path), exist_ok=True)
    with open(output_h_path, "w") as f:
        f.write("/* SPDX-License-Identifier: GPL-2.0+ */\n")
        f.write("/*\n")
        f.write(
            f" * Auto-generated Clean-Room PMU Firmware Header "
            f"({len(hwords) * 2} bytes / {len(hwords)} halfwords).\n"
        )
        f.write(" */\n\n")
        f.write("#ifndef __DRAM_SUN60I_A733_PMU_FW_H__\n")
        f.write("#define __DRAM_SUN60I_A733_PMU_FW_H__\n\n")
        f.write("#include <linux/types.h>\n\n")
        f.write(f"static const u16 sun60i_a733_pmu_imem[{len(hwords)}] = {{\n")
        for i in range(0, len(hwords), 8):
            hex_str = ", ".join(f"0x{x:04x}" for x in hwords[i:i + 8])
            f.write(f"\t{hex_str},\n")
        f.write("};\n\n")
        f.write("#endif /* __DRAM_SUN60I_A733_PMU_FW_H__ */\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
