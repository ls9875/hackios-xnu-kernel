#!/bin/bash
# Turn the OnePlus 8T (kebab, SM8250) Mu-Silicium platform into one for the Xiaomi Mi 10 (umi, SM8250).
#
# Run from the workspace root, after uefi/apply.sh has made ../Mu-Silicium:
#     uefi/umi-port.sh
#
# Why kebab: it is the only SM8250 device in Mu-Silicium, and SM8250 is what umi is (msm-id 0x164 in
# the phone's own device tree). The SoC include, SOC_TYPE and the ACPI tables all come from kebab; what
# this changes is the device's own identity, the panel height (umi is 1080x2340, kebab 1080x2400) and
# the memory map (step 1b).
#
# What it does NOT do: replace the UEFI driver binaries with ones extracted from umi's own xbl.img.
# Binaries/kebab is the first attempt; if the phone shows nothing, that extraction is the next step
# (see PORT-UMI.md).
#
# Order matters, and it used to be wrong: uefi/apply.sh runs first and has already created
# Platforms/Xiaomi/umiPkg out of uefi/files/, so "if the directory does not exist, build it" skipped
# every copy below and the build then failed with
#     The Device Build Script isn't Present in "Platforms/Xiaomi/umiPkg"!
# The copies are therefore unconditional, and no-clobber: the files this repo carries win, kebab only
# fills the gaps.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

die() { echo "umi-port: $*" >&2; exit 1; }

MU=Mu-Silicium
UMI=$MU/Platforms/Xiaomi/umiPkg
KEBAB=$MU/Platforms/OnePlus/kebabPkg

[ -d $MU ]    || die "no $MU -- run uefi/apply.sh first"
[ -d $KEBAB ] || die "kebabPkg missing in $MU -- uefi/apply.sh should have cloned Mu-Silicium"

# ---------------------------------------------------------------------------------------------
# 1. the platform package: kebab's driver list and libraries, this repo's own .dsc/.fdf/.dec
# ---------------------------------------------------------------------------------------------
mkdir -p $UMI/Include $UMI/Library
cp -rn $KEBAB/Include/.      $UMI/Include/
cp -rn $KEBAB/Library/.      $UMI/Library/
cp -n  $KEBAB/DeviceBuild.py $UMI/DeviceBuild.py

# The copied files name the donor package in their paths (Binaries/kebab/..., kebabPkg/...).
# umi.dsc / umi.fdf are deliberately left out: they point at kebab's ACPI tables on purpose, because
# Silicium-ACPI has no umi directory -- rewriting that to umi/AcpiTables.inf would break the build.
# mapfile rather than a pipe: grep exits 1 when there is nothing to rename, which is exactly what a
# second run looks like, and pipefail would turn that into a failed job.
mapfile -d '' rename < <(grep -rlZ --exclude=umi.dsc --exclude=umi.fdf --exclude=umi.dec 'kebab' $UMI || true)
if [ ${#rename[@]} -gt 0 ]; then
	sed -i 's/kebabPkg/umiPkg/g; s/kebab/umi/g' "${rename[@]}"
	echo "umi-port: renamed the donor package in ${#rename[@]} copied file(s)"
fi

# 1b. the memory map. kebab's is right for umi in every region the phone's device tree mentions (its
#     "Display Reserved" 0x9C000000+0x2400000 ends exactly where umi's splash+dfps reservations end,
#     0x9E400000), but its DXE heap runs 0xB0000000+0x4500000 and so covers 0xF0000000, which is where
#     the loader puts the XNU payload (PAYLOAD_BASE, hardcoded in xnuloader/loader.c and in
#     SiliciumPkg/Library/BootManagerLib/XnuLoader.h). uefi/files/ carries umi's corrected copy; the
#     no-clobber copy above left it in place. Fail loudly if it ever goes missing, because the symptom
#     would be a UEFI that boots and then hands the kernel a heap full of firmware.
[ -f $UMI/Library/MemoryMapLib/MemoryMapLib.c ] || die "umiPkg/Library/MemoryMapLib/MemoryMapLib.c missing"
grep -q 'XNU Payload' $UMI/Library/MemoryMapLib/MemoryMapLib.c \
	|| die "MemoryMapLib.c has no XNU Payload region -- 0xF0000000 would land in the DXE heap"

# ---------------------------------------------------------------------------------------------
# 2. the UEFI driver binaries of the donor device (Include/*.inc name Binaries/umi/... after step 1)
# ---------------------------------------------------------------------------------------------
if [ ! -d $MU/Binaries/umi ]; then
	[ -d $MU/Binaries/kebab ] || die "Binaries/kebab missing (submodule not checked out?)"
	cp -r $MU/Binaries/kebab $MU/Binaries/umi
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
grep -q 'Binaries/umi/' $UMI/Include/DXE.inc || die "DXE.inc still points at Binaries/kebab"
grep -q 'kebab/AcpiTables.inf' $UMI/umi.dsc || die "umi.dsc lost its kebab/AcpiTables.inf reference"
grep -q 'FLASH_DEFINITION *= *umiPkg/umi.fdf' $UMI/umi.dsc || die "umi.dsc does not use umi.fdf"

echo "umi-port: Platforms/Xiaomi/umiPkg ready"
echo "umi-port:   DeviceBuild.py   $(stat -c %s $UMI/DeviceBuild.py) bytes"
echo "umi-port:   DXE.inc          $(grep -c 'INF ' $UMI/Include/DXE.inc) driver lines"
echo "umi-port:   Binaries/umi     $(find $MU/Binaries/umi -name '*.efi' | wc -l) .efi"
echo "umi-port:   SOC_TYPE         $(sed -n 's/^ *SOC_TYPE *= *//p' $UMI/umi.dsc)"
grep -m1 'PcdFrameBufferWidth'  $UMI/umi.dsc
grep -m1 'PcdFrameBufferHeight' $UMI/umi.dsc
echo "umi-port:   FD base/size     $(sed -n 's/^ *base *= *//p' $MU/Resources/Configs/umi.toml) / $(sed -n 's/^ *size *= *//p' $MU/Resources/Configs/umi.toml)"
echo "umi-port: next: python3 build_uefi.py -d umi -r RELEASE (inside $MU)"
