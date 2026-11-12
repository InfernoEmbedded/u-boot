#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
# Copyright (C) 2026 Alastair D'Silva <alastair@d-silva.org>
"""
Single-pass ELF builder for Allwinner A733 LPDDR5 ARC EM4 PMU Firmware.
"""

import argparse
import configparser
import glob
import os
import shutil
import struct
import subprocess
import sys
import tempfile

DEFAULT_SOURCES = (
    "pmu_primitives.c",
    "pmu_clock.c",
    "pmu_telemetry.c",
    "pmu_cbt.c",
    "pmu_deskew.c",
    "pmu_eye.c",
    "pmu_train.c",
)


def resolve_arc_toolchain(preferred_prefix, explicit=False):
    """Locate an ARC GCC toolchain prefix across PATH and U-Boot CI locations."""
    if preferred_prefix:
        if shutil.which(f"{preferred_prefix}gcc") and shutil.which(f"{preferred_prefix}objcopy"):
            return preferred_prefix
        if explicit:
            raise RuntimeError(
                f"Explicitly requested ARC toolchain '{preferred_prefix}gcc' not found"
            )

    candidates = [
        "arc-linux-gnu-",
        "arc-linux-",
        "arc-elf32-",
    ]
    for bman_cfg in (
        os.path.expanduser("~/.buildman"),
        "/home/uboot/.buildman",
    ):
        if not os.path.isfile(bman_cfg):
            continue
        cfg = configparser.ConfigParser()
        try:
            cfg.read(bman_cfg)
            if cfg.has_option("toolchain-prefix", "arc"):
                candidates.append(os.path.expanduser(cfg.get("toolchain-prefix", "arc")))
            if cfg.has_section("toolchain"):
                for _, root in cfg.items("toolchain"):
                    root = os.path.expanduser(root)
                    for pat in ("bin/arc-*-gcc", "*/bin/arc-*-gcc"):
                        candidates.extend(sorted(glob.glob(os.path.join(root, pat)), reverse=True))
        except configparser.Error:
            pass

    for pattern in (
        "/opt/gcc-*-nolibc/arc-linux/bin/arc-linux-",
        os.path.expanduser("~/.buildman-toolchains/gcc-*-nolibc/arc-linux/bin/arc-linux-"),
        "/home/uboot/.buildman-toolchains/gcc-*-nolibc/arc-linux/bin/arc-linux-",
    ):
        candidates.extend(sorted(glob.glob(pattern), reverse=True))

    for prefix in candidates:
        if prefix.endswith("gcc"):
            prefix = prefix[:-3]
        if shutil.which(f"{prefix}gcc") and shutil.which(f"{prefix}objcopy"):
            return prefix

    return None


def write_header(output_h_path, hwords, missing=False):
    """Atomically emit the C header containing the PMU ICCM image or missing stub."""
    out_dir = os.path.dirname(output_h_path)
    os.makedirs(out_dir, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=out_dir, delete=False, prefix=".pmu_fw_tmp_") as f:
        tmp_name = f.name
        f.write("/* SPDX-License-Identifier: GPL-2.0+ */\n")
        f.write("/*\n")
        if missing:
            f.write(" * Auto-generated stub ARC EM4 PMU Firmware Header (missing toolchain).\n")
        else:
            f.write(
                f" * Auto-generated ARC EM4 PMU Firmware Header "
                f"({len(hwords) * 2} bytes / {len(hwords)} halfwords).\n"
            )
        f.write(" */\n\n")
        f.write("#ifndef __DRAM_SUN60I_A733_PMU_FW_H__\n")
        f.write("#define __DRAM_SUN60I_A733_PMU_FW_H__\n\n")
        f.write("#include <linux/compiler_attributes.h>\n")
        f.write("#include <linux/types.h>\n\n")
        if missing:
            f.write("#define SUN60I_A733_PMU_FW_MISSING 1\n\n")
            f.write(
                "static const u16 __section(\".init.rodata\") "
                "sun60i_a733_pmu_imem[1] = { 0 };\n\n"
            )
        else:
            f.write(
                f"static const u16 __section(\".init.rodata\") "
                f"sun60i_a733_pmu_imem[{len(hwords)}] = {{\n"
            )
            for i in range(0, len(hwords), 8):
                hex_str = ", ".join(f"0x{x:04x}" for x in hwords[i:i + 8])
                f.write(f"\t{hex_str},\n")
            f.write("};\n\n")
        f.write("#endif /* __DRAM_SUN60I_A733_PMU_FW_H__ */\n")

    os.replace(tmp_name, output_h_path)


def main():
    pmu_dir_default = os.path.abspath(os.path.dirname(__file__))
    repo_root = os.path.abspath(os.path.join(pmu_dir_default, "..", "..", "..", ".."))
    default_out = os.path.join(repo_root, "arch/arm/mach-sunxi/dram_sun60i_a733_pmu_fw.h")

    parser = argparse.ArgumentParser(
        description="Build Allwinner A733 LPDDR5 ARC EM4 PMU firmware."
    )
    parser.add_argument("--cross-compile", default="", help="ARC toolchain prefix")
    parser.add_argument("--pmu-fw", default="", help="Prebuilt raw PMU ICCM binary path")
    parser.add_argument("--allow-missing", action="store_true",
                        help="Emit a non-functional stub header if ARC toolchain is absent")
    parser.add_argument("--src-dir", default=pmu_dir_default, help="PMU firmware source directory")
    parser.add_argument("--ld-script", default="", help="Linker script path")
    parser.add_argument("--out", default=default_out, help="Output C header path")
    parser.add_argument("sources", nargs="*", help="Explicit C source files to compile")
    args = parser.parse_args()

    src_dir = os.path.abspath(args.src_dir)
    ld_script = os.path.abspath(args.ld_script) if args.ld_script else os.path.join(src_dir, "pmu.ld")
    output_h_path = os.path.abspath(args.out)

    if args.pmu_fw:
        with open(args.pmu_fw, "rb") as bf:
            raw = bf.read()
        if not raw:
            raise ValueError(f"PMU firmware '{args.pmu_fw}' is empty")
        if len(raw) > 0x10000:
            raise ValueError(
                f"PMU firmware '{args.pmu_fw}' exceeds 64 KiB ICCM limit ({len(raw)} bytes)"
            )
    else:
        explicit_cc = any(arg.startswith("--cross-compile") for arg in sys.argv[1:])
        prefix = resolve_arc_toolchain(args.cross_compile, explicit=explicit_cc)
        if prefix is None:
            if args.allow_missing:
                print(
                    "Image 'spl/u-boot-spl.bin' is missing ARC cross-compiler; "
                    "using non-functional stub PMU firmware",
                    file=sys.stderr,
                )
                write_header(output_h_path, [], missing=True)
                return 0
            print(
                "Error: ARC toolchain not found (install arc-linux-gnu-gcc, pass PMU_FW=, "
                "or build with BINMAN_ALLOW_MISSING=1)",
                file=sys.stderr,
            )
            return 1

        gcc = f"{prefix}gcc"
        objcopy = f"{prefix}objcopy"
        c_files = (
            [os.path.abspath(p) for p in args.sources]
            if args.sources
            else [os.path.join(src_dir, name) for name in DEFAULT_SOURCES]
        )

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
            "-ffreestanding",
            "-fno-caller-saves",
            "-fno-ipa-ra",
            "-ffixed-r26",
            "-ffixed-r27",
            "-fno-strict-aliasing",
            "-fno-jump-tables",
            "-fno-tree-switch-conversion",
            "-fno-pic",
            "-fno-pie",
            "-fno-stack-protector",
            "-fno-builtin",
            "-ffunction-sections",
            "-fdata-sections",
            "-std=gnu11",
            "-Wall",
            "-Wextra",
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
                "-no-pie",
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

    write_header(output_h_path, hwords, missing=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
