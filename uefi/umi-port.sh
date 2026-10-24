#!/bin/bash
# Turn the OnePlus 8T (kebab, SM8250) Mu-Silicium platform into one for the Xiaomi Mi 10 (umi, SM8250).
#
# Run from the workspace root, after uefi/apply.sh has made ../Mu-Silicium:
#     uefi/umi-port.sh
#
# Why kebab: it is the only SM8250 device in Mu-Silicium, and SM8250 is what umi is (msm-id 0x164 in
# the phone's own device tree). The SoC include, SOC_TYPE and the ACPI tables all come from kebab; what
# this changes is the device's own identity and the panel height (umi is 1080x2340, kebab 1080x2400).
#
# What it does NOT do: replace the UEFI driver binaries with ones extracted from umi's own xbl.img.
# Binaries/kebab is the first attempt; if the phone shows nothing, that extraction is the next step
# (see PORT-UMI.md).
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."
MU=Mu-Silicium
[ -d $MU ] || { echo "umi-port: no $MU -- run uefi/apply.sh first" >&2; exit 1; }
[ -d $MU/Platforms/OnePlus/kebabPkg ] || { echo "umi-port: kebabPkg missing in $MU" >&2; exit 1; }

# 1. the platform package: kebab's libraries and driver list, our own .dsc/.fdf/.dec
if [ ! -d $MU/Platforms/Xiaomi/umiPkg ]; then
	mkdir -p $MU/Platforms/Xiaomi/umiPkg
	cp -r $MU/Platforms/OnePlus/kebabPkg/Include      $MU/Platforms/Xiaomi/umiPkg/
	cp -r $MU/Platforms/OnePlus/kebabPkg/Library      $MU/Platforms/Xiaomi/umiPkg/
	cp    $MU/Platforms/OnePlus/kebabPkg/DeviceBuild.py $MU/Platforms/Xiaomi/umiPkg/
	# Library/*/*.inf name their package; keep them pointing at umiPkg
	grep -rl 'kebabPkg' $MU/Platforms/Xiaomi/umiPkg | while read -r f; do
		sed -i 's/kebabPkg/umiPkg/g; s/kebab/umi/g' "$f"
	done
fi
# uefi/files/ (copied in by uefi/apply.sh) already put our umi.dsc/.fdf/.dec there; if a file is
# missing, fall back to kebab's with the name changed so a build can still be attempted.
for ext in dsc fdf dec; do
	if [ ! -f $MU/Platforms/Xiaomi/umiPkg/umi.$ext ]; then
		echo "umi-port: umi.$ext missing, deriving it from kebab.$ext" >&2
		sed -e 's/kebabPkg/umiPkg/g' -e 's/kebab/umi/g' \
			$MU/Platforms/OnePlus/kebabPkg/kebab.$ext > $MU/Platforms/Xiaomi/umiPkg/umi.$ext
	fi
done

# 2. the UEFI driver binaries of the donor device
if [ ! -d $MU/Binaries/umi ]; then
	[ -d $MU/Binaries/kebab ] || { echo "umi-port: Binaries/kebab missing (submodule not checked out?)" >&2; exit 1; }
	cp -r $MU/Binaries/kebab $MU/Binaries/umi
fi

# 3. the device tree and the boot shim config: uefi/apply.sh copies everything under uefi/files/
#    into Mu-Silicium, so these are only checked, not written.
for f in Resources/DTBs/umi.dtb Resources/Configs/umi.toml; do
	[ -f "$MU/$f" ] || { echo "umi-port: $f missing -- uefi/apply.sh should have copied uefi/files/$f" >&2; exit 1; }
done
echo "umi-port: device tree $(stat -c %s "$MU/Resources/DTBs/umi.dtb") bytes"

echo "umi-port: Platforms/Xiaomi/umiPkg ready"
echo "umi-port: SOC_TYPE=$(sed -n 's/^ *SOC_TYPE *= *//p' $MU/Platforms/Xiaomi/umiPkg/umi.dsc)"
grep -m1 'PcdFrameBufferWidth' $MU/Platforms/Xiaomi/umiPkg/umi.dsc
grep -m1 'PcdFrameBufferHeight' $MU/Platforms/Xiaomi/umiPkg/umi.dsc
echo "umi-port: next: python3 build_uefi.py -d umi -r RELEASE (inside $MU)"
