# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2000-2002
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# Copyright (c) 2017 Microsemi Corporation.
# Padmarao Begari, Microsemi Corporation <padmarao.begari@microsemi.com>
#
# Copyright (C) 2017 Andes Technology Corporation
# Rick Chen, Andes Technology Corporation <rick@andestech.com>
#

CONFIG_32BIT := "1"

CROSS_COMPILE := riscv64-unknown-elf-

PLATFORM_LDFLAGS	+= -m elf32lriscv
EFI_LDS			:= elf_riscv32_efi.lds

CONFIG_STANDALONE_LOAD_ADDR = 0x08100000 \
			      -T $(srctree)/examples/standalone/riscv.lds

PLATFORM_RELFLAGS 	+= -fno-strict-aliasing -fno-common -gdwarf-2 -ffunction-sections
PLATFORM_CPPFLAGS	+= -ffixed-gp -mcmodel=medany -fpic -march=rv32im -mabi=ilp32

LDFLAGS_u-boot = --gc-sections -static

EFI_CRT0		:= crt0_riscv_efi.o
EFI_RELOC		:= reloc_riscv_efi.o
