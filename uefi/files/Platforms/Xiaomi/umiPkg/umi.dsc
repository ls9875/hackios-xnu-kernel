##
#  Xiaomi Mi 10 (umi) -- Snapdragon 865 / SM8250 "kona", for HackiOS.
#
#  Derived from Platforms/OnePlus/kebabPkg (OnePlus 8T), the only SM8250 device in Mu-Silicium:
#  the SoC include (KonaPkg), SOC_TYPE and the ACPI tables come from there unchanged. What differs is
#  this device's identity and the panel: umi is 1080x2340, kebab 1080x2400.
#
#  Copyright (c) 2011 - 2022, ARM Limited. All rights reserved.
#  Copyright (c) 2014, Linaro Limited. All rights reserved.
#  Copyright (c) 2015 - 2020, Intel Corporation. All rights reserved.
#  Copyright (c) 2018, Bingxing Wang. All rights reserved.
#  Copyright (c) Microsoft Corporation.
#
#  SPDX-License-Identifier: BSD-2-Clause-Patent
##

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = umi
  PLATFORM_GUID                  = A7F3C1D2-6B4E-4E8A-9C15-3D7E2B9F4A61
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010005
  OUTPUT_DIRECTORY               = Build/umiPkg
  SUPPORTED_ARCHITECTURES        = AARCH64
  BUILD_TARGETS                  = RELEASE|DEBUG
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = umiPkg/umi.fdf
  USE_CUSTOM_DISPLAY_DRIVER      = 0

  #
  # 0 = SM8250
  # 1 = SM8250-AB
  # 2 = SM8250-AC
  #
  # Mi 10 is plain SM8250 -- the same value kebab (OnePlus 8T) uses.
  #
  SOC_TYPE                       = 0

!include KonaPkg/KonaPkg.dsc.inc

[PcdsFixedAtBuild]
  #
  # DDR Memory
  #
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x80000000

  #
  # UEFI Stack
  #
  gArmPlatformTokenSpaceGuid.PcdCPUCoresStackBase|0x9FF90000
  gArmPlatformTokenSpaceGuid.PcdCPUCorePrimaryStackSize|0x40000

  #
  # SMBIOS
  #
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemManufacturer|"Xiaomi"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemModel|"Mi 10"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemRetailModel|"umi"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemRetailSku|"Mi_10_umi"
  gSiliciumPkgTokenSpaceGuid.PcdSmbiosSystemBoardModel|"Mi 10"

  #
  # Simple Frame Buffer
  #
  # 1080x2340: read from the phone (wm size), and the panel the device tree describes.
  #
  gSiliciumPkgTokenSpaceGuid.PcdFrameBufferWidth|1080
  gSiliciumPkgTokenSpaceGuid.PcdFrameBufferHeight|2340
  gSiliciumPkgTokenSpaceGuid.PcdFrameBufferColorDepth|32

  #
  # Platform PEI
  #
  gQcomPkgTokenSpaceGuid.PcdPlatformType|"LA"

[LibraryClasses]
  #
  # Memory Libraries
  #
  MemoryMapLib|umiPkg/Library/MemoryMapLib/MemoryMapLib.inf

  #
  # QCOM Libraries
  #
  ConfigurationMapLib|umiPkg/Library/ConfigurationMapLib/ConfigurationMapLib.inf

[Components]
  #
  # ACPI Tables
  #
  # kebab's tables: same SoC, so the same interrupt controller, timers and UARTs.
  #
  kebab/AcpiTables.inf
