#!/bin/bash
# Turn the Xiaomi POCO F3 (alioth, SM8250-AC) Mu-Silicium platform into one for the Xiaomi Mi 10
# (umi, SM8250).
#
# Run from the workspace root, after uefi/apply.sh has made ../Mu-Silicium:
#     uefi/umi-port.sh
#
# Why alioth and not kebab: the first version of this port was built on OnePlus/kebabPkg (OnePlus 8T),
# the only SM8250 device in Mu-Silicium at the time, and it did not come up on the phone at all --
# black screen, nothing on USB, back to the bootloader. The reason is step 2 below: Binaries/<device>
# is the UEFI driver set extracted from that phone's own xbl.img, so kebab's copy is OnePlus 8T
# firmware driving a Mi 10's panel, PMIC and USB PHY.
#
# alioth is the same silicon (Snapdragon 870 is SM8250-AC: SOC_TYPE 2 against umi's 0) from the same
# vendor, so its driver set is Xiaomi's own. Its memory map also matches umi's device tree region for
# region -- not approximately, exactly:
#
#     alioth map                                  umi's own reserved-memory
#     HYP              0x80000000+0x600000    ==  hyp_region@80000000
#     AOP CMD DB       0x80860000+0x20000     ==  reserved-memory@80860000
#     SMEM             0x80900000+0x200000    ==  smem_region@80900000
#     QTEE + TZApps    0x80B00000+0x5300000   ==  removed_region@80b00000
#     PIL Reserved     0x86200000+0xC500000   ==  the pil_*_region cluster
#     Display Reserved 0x9C000000+0x2400000   ==  cont_splash_region + dfps_data_region
#     UEFI FD          0x9FC00000+0x300000    ==  the FD region this port uses
#
# What is still this port's own: the device identity, the panel height (umi is 1080x2340, alioth
# 1080x2400), SOC_TYPE 0 instead of 2, and the XNU Payload reservation in the memory map (step 1b).
#
# Order matters, and it used to be wrong: uefi/apply.sh runs first and has already created
# Platforms/Xiaomi/umiPkg out of uefi/files/, so "if the directory does not exist, build it" skipped
# every copy below and the build then failed with
#     The Device Build Script isn't Present in "Platforms/Xiaomi/umiPkg"!
# The copies are therefore unconditional, and no-clobber: the files this repo carries win, alioth
# only fills the gaps.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

die() { echo "umi-port: $*" >&2; exit 1; }

MU=Mu-Silicium
UMI=$MU/Platforms/Xiaomi/umiPkg
ALIOTH=$MU/Platforms/Xiaomi/aliothPkg

[ -d $MU ]     || die "no $MU -- run uefi/apply.sh first"
[ -d $ALIOTH ] || die "aliothPkg missing in $MU -- uefi/apply.sh should have cloned Mu-Silicium"

# ---------------------------------------------------------------------------------------------
# 1. the platform package: alioth's driver list and libraries, this repo's own .dsc/.fdf
# ---------------------------------------------------------------------------------------------
mkdir -p $UMI/Include $UMI/Library
cp -rn $ALIOTH/Include/.      $UMI/Include/
cp -rn $ALIOTH/Library/.      $UMI/Library/
cp -n  $ALIOTH/DeviceBuild.py $UMI/DeviceBuild.py

# The copied files name the donor package in their paths (Binaries/alioth/..., aliothPkg/...).
# umi.dsc / umi.fdf are deliberately left out: they point at alioth's ACPI tables on purpose, because
# Silicium-ACPI has no umi directory -- rewriting that to umi/AcpiTables.inf would break the build.
# mapfile rather than a pipe: grep exits 1 when there is nothing to rename, which is exactly what a
# second run looks like, and pipefail would turn that into a failed job.
mapfile -d '' rename < <(grep -rlZ --exclude=umi.dsc --exclude=umi.fdf 'alioth' $UMI || true)
if [ ${#rename[@]} -gt 0 ]; then
	sed -i 's/aliothPkg/umiPkg/g; s/alioth/umi/g' "${rename[@]}"
	echo "umi-port: renamed the donor package in ${#rename[@]} copied file(s)"
fi

# DeviceBuild.py lists the directories EDK2 resolves a package name in. alioth's already has
# Platforms/Xiaomi (it is where aliothPkg lives), so this is a guard rather than a fix -- but the
# failure it prevents is obscure and silent until the end of setup:
#     ERROR - Failed to convert Edk2Relative Path to an Absolute Path on this system.
#     ERROR - Relative Path: umiPkg/umi.dsc
# Silicium-ACPI/Platforms/Xiaomi stays: it is where the alioth/AcpiTables.inf umi.dsc asks for lives.
if ! grep -q '"Platforms/Xiaomi"' $UMI/DeviceBuild.py; then
	sed -i 's|^\( *\)PackagesPath = (|\1PackagesPath = (\n\1    "Platforms/Xiaomi",|' $UMI/DeviceBuild.py
fi

# alioth's DeviceBuild.py forwards DEVICE_MODEL, the switch that picks POCO F3 / Redmi K40 / Mi 11X
# SMBIOS strings. umi.dsc has no such switch and build_uefi.py is never given the variable, so the
# line would push a build var whose value is None. Drop it rather than depend on what edk2toolext
# does with that, and prove the .dsc really does not want it.
sed -i '/BLD_\*_DEVICE_MODEL/d' $UMI/DeviceBuild.py
if grep -q 'DEVICE_MODEL' $UMI/umi.dsc; then
	die "umi.dsc now uses DEVICE_MODEL -- stop dropping it from DeviceBuild.py"
fi

# 1b. the memory map. alioth's is right for umi in every region the phone's device tree mentions (the
#     table at the top of this file is that comparison), but it has no entry for PAYLOAD_BASE: 0xF0000000
#     is simply not in the map, so nothing should touch it -- "should" being the word that makes a
#     reservation worth the four lines. uefi/files/ carries umi's copy with the region added; the
#     no-clobber copy above left it in place. Fail loudly if it ever goes missing, because the symptom
#     would be a UEFI that boots and then hands the kernel a payload full of firmware.
[ -f $UMI/Library/MemoryMapLib/MemoryMapLib.c ] || die "umiPkg/Library/MemoryMapLib/MemoryMapLib.c missing"
grep -q 'XNU Payload' $UMI/Library/MemoryMapLib/MemoryMapLib.c \
	|| die "MemoryMapLib.c has no XNU Payload region -- 0xF0000000 would be unaccounted for"

# ---------------------------------------------------------------------------------------------
# 2. the UEFI driver binaries of the donor device (Include/*.inc name Binaries/umi/... after step 1)
# ---------------------------------------------------------------------------------------------
if [ ! -d $MU/Binaries/umi ]; then
	[ -d $MU/Binaries/alioth ] || die "Binaries/alioth missing (submodule not checked out?)"
	cp -r $MU/Binaries/alioth $MU/Binaries/umi
fi
[ -d $MU/Binaries/umi/QcomPkg ] || die "Binaries/umi has no QcomPkg -- the driver copy did not take"

# ---------------------------------------------------------------------------------------------
# 3. the device tree and the boot shim config: uefi/apply.sh copies everything under uefi/files/
#    into Mu-Silicium, so these are only checked, not written.
# ---------------------------------------------------------------------------------------------
for f in Resources/DTBs/umi.dtb Resources/Configs/umi.toml; do
	[ -f "$MU/$f" ] || die "$f missing -- uefi/apply.sh should have copied uefi/files/$f"
done
echo "umi-port: device tree $(stat -c %s "$MU/Resources/DTBs/umi.dtb") bytes"

# ---------------------------------------------------------------------------------------------
# 4. prove the things build_uefi.py -d umi actually needs are now in place
# ---------------------------------------------------------------------------------------------
for f in DeviceBuild.py Include/DXE.inc Library/MemoryMapLib/MemoryMapLib.inf \
         Library/ConfigurationMapLib/ConfigurationMapLib.inf; do
	[ -f "$UMI/$f" ] || die "umiPkg/$f missing"
done
grep -q 'Binaries/umi/' $UMI/Include/DXE.inc || die "DXE.inc still points at Binaries/alioth"
grep -q 'alioth/AcpiTables.inf' $UMI/umi.dsc || die "umi.dsc lost its alioth/AcpiTables.inf reference"
grep -q 'alioth/AcpiTables.inf' $UMI/umi.fdf || die "umi.fdf lost its alioth/AcpiTables.inf reference"
grep -q 'FLASH_DEFINITION *= *umiPkg/umi.fdf' $UMI/umi.dsc || die "umi.dsc does not use umi.fdf"
grep -q '"Platforms/Xiaomi"' $UMI/DeviceBuild.py || die "DeviceBuild.py does not resolve packages in Platforms/Xiaomi"
grep -q 'ACTIVE_PLATFORM", "umiPkg/umi.dsc' $UMI/DeviceBuild.py || die "DeviceBuild.py still builds another platform"
# The USB drivers the loader asks for by protocol GUID (QCOM_USB_DEVICE_GUID, usb_start() in
# xnuloader/loader.c). Without them the loader prints "usb: device protocol not found" and resets.
for d in UsbDeviceDxe UsbInitDxe UsbfnDwc3Dxe; do
	grep -q "Binaries/umi/QcomPkg/Drivers/$d/" $UMI/Include/DXE.inc || die "DXE.inc has no $d"
done

echo "umi-port: Platforms/Xiaomi/umiPkg ready"
echo "umi-port:   DeviceBuild.py   $(stat -c %s $UMI/DeviceBuild.py) bytes"
echo "umi-port:   DXE.inc          $(grep -c 'INF ' $UMI/Include/DXE.inc) driver lines"
echo "umi-port:   Binaries/umi     $(find $MU/Binaries/umi -name '*.efi' | wc -l) .efi (from alioth)"
echo "umi-port:   SOC_TYPE         $(sed -n 's/^ *SOC_TYPE *= *//p' $UMI/umi.dsc)"
grep -m1 'PcdFrameBufferWidth'  $UMI/umi.dsc
grep -m1 'PcdFrameBufferHeight' $UMI/umi.dsc
echo "umi-port:   FD base/size     $(sed -n 's/^ *base *= *//p' $MU/Resources/Configs/umi.toml) / $(sed -n 's/^ *size *= *//p' $MU/Resources/Configs/umi.toml)"
echo "umi-port: next: python3 build_uefi.py -d umi -r RELEASE (inside $MU)"
