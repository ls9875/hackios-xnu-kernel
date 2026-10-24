# Xiaomi Mi 10 (umi) — port notes

Target: HackiOS / xnu-8796.121.2 (iOS 16.5) running on a Xiaomi Mi 10 (`umi`, SM8250 "kona",
Snapdragon 865), entirely from RAM: `fastboot boot <image>`, nothing written to storage, a power-key
reset brings Android back.

umi is **not** a device HackiOS or Mu-Silicium supports upstream. The donor is:

| what | donor | why |
|---|---|---|
| UEFI (Mu-Silicium) | `Xiaomi/aliothPkg` (POCO F3 / Redmi K40, SM8250-AC) | same silicon, same vendor |
| UEFI drivers (`Binaries/`) | `Binaries/alioth` | Xiaomi's own xbl drivers, not another vendor's |
| memory map | same | matches umi's own `/reserved-memory` region for region |
| ACPI tables | `Silicium-ACPI/Platforms/Xiaomi/alioth` | same SoC family |

Snapdragon 870 *is* SM8250-AC — the same die as the 865 at higher clocks, which is exactly what the
`SOC_TYPE` switch in the DSC is for (`0` = SM8250, `2` = SM8250-AC). umi keeps `0`.

**Why not OnePlus 8T (`kebab`), which was the first attempt.** kebab was the only SM8250 device in
Mu-Silicium when this port started, so the platform package and `Binaries/kebab` were copied from it.
The image loaded (`Sending OKAY` / `Booting OKAY`) and then the phone went dark: no framebuffer, no
USB enumeration at any point, back to the bootloader after a minute. `Binaries/<device>` is the UEFI
driver set extracted from *that phone's* `xbl.img`, so kebab's copy is OnePlus 8T firmware driving a
Mi 10's panel, PMIC and USB PHY. alioth fixes that: a Xiaomi device's own drivers.

## Verified from the phone

Everything below came out of the phone's own device tree (`/sys/firmware/fdt`, dumped over adb), not
from a guess and not from another device's profile.

| value | where it comes from | value in umi.conf |
|---|---|---|
| UEFI FD base | `MemoryMapLib.c` "UEFI FD" | `FD_BASE=0x9FC00000` |
| UEFI FD size | same, and `umi.toml [uefi_fd]` | `FD_SIZE=0x300000` |
| XNU payload base | `MemoryMapLib.c` "XNU Payload", also hardcoded in `xnuloader/loader.c` | `PAYLOAD_BASE=0xF0000000` |
| framebuffer base | base of the map's "Display Reserved" (the relationship redwood has: `0xE1000000`) | `FB_BASE=0x9C000000` |

`tools/dtb-mem.py` in the workspace parses the dtb and answers the question that matters for a fixed
load address: is it free? All three of the above come back free (and `0x9C000000` comes back as
`cont_splash_region` + `disp_rdump_region`, which is what a framebuffer base should be).

alioth's map and umi's device tree, side by side:

| alioth map | umi's `/reserved-memory` | |
|---|---|---|
| `HYP 0x80000000+0x600000` | `hyp_region@80000000 +0x600000` | same |
| `AOP CMD DB 0x80860000+0x20000` | `reserved-memory@80860000 +0x20000` | same |
| `SMEM 0x80900000+0x200000` | `smem_region@80900000 +0x200000` | same |
| `QTEE+TZApps 0x80B00000+0x5300000` | `removed_region@80b00000 +0x5300000` | same, exactly |
| `PIL Reserved 0x86200000+0xC500000` | the `pil_*_region` cluster | same, ends `0x92700000` |
| `Display Reserved 0x9C000000+0x2400000` | `cont_splash` + `dfps_data` | same, ends `0x9E400000` |
| `UEFI FD 0x9FC00000+0x300000` | free DDR above the splash | free |

## What was changed to make a umi UEFI

1. `uefi/umi-port.sh` — derives `Platforms/Xiaomi/umiPkg` from `aliothPkg` (copies, renames
   `aliothPkg`→`umiPkg` / `alioth`→`umi`, copies `Binaries/alioth`→`Binaries/umi`, keeps
   `alioth/AcpiTables.inf` since the ACPI tables live under `Silicium-ACPI/Platforms/Xiaomi`). Ends
   with assertions that fail the job rather than producing a plausible-looking but wrong FD —
   including one that the three USB drivers the loader asks for by protocol GUID are in the driver
   list (`UsbDeviceDxe`, `UsbInitDxe`, `UsbfnDwc3Dxe`).
2. `umiPkg/Library/MemoryMapLib/MemoryMapLib.c` — alioth's table plus `{"XNU Payload", 0xF0000000,
   0x04000000}`.
3. `Resources/Configs/umi.toml` — `FD_BASE`/`FD_SIZE` from the table above; no `[boot_image]`
   section, so the UEFI does not try to pack Mu's own ramdisk.
4. `umi.dsc` — `SOC_TYPE = 0`, 1080×2340, Mi 10 identity, `alioth/AcpiTables.inf`.
5. `umi.fdf` — the DTB is not embedded in the FD; the device tree travels in the boot image.
6. `common/devices/umi.conf`, `common/devices/umi.platform.c`, `xnuloader/loader.c` — the addresses
   above, `platform_umi` selected by `fb_base`.

## Building it

No Linux toolchain on the machine this was ported on, so the whole build runs in GitHub Actions
(`.github/workflows/umi.yml`, `ubuntu-latest`), on a fork at `ls9875/hackios-xnu-kernel`.
Two inputs on manual dispatch: `uefi` (30 min) and `images` (up to 5 h).

The boot image itself is small and can be rebuilt locally once the UEFI and the loader exist:

    common/build.sh umi            # writes out/.device, then out/umi-boot.img
    ./xl enter                     # same image, and puts the phone into the loader

`prebuilt/uefi/umi/` holds the CI-produced `SILICIUM_UEFI.fd` (3 MB), `BootShim.bin` and `umi.dtb`,
so `mkxnuimg.py` can build umi images without a Mu-Silicium tree. It looks for that directory by
target name; redwood keeps using `prebuilt/uefi/` itself.

`BootShim.bin` is the part that must not be mixed up: it is compiled with `FD_BASE`/`FD_SIZE` baked
in, so redwood's shim (`0x9FD00000`/`0x200000`) copies a 3 MB FD to the wrong address and the phone
shows nothing at all. Word 2 of the shim is FD_BASE and word 4 is FD_SIZE — `0x9fc00000`/`0x300000`
is umi, `0x9fd00000`/`0x200000` is redwood.

## Booting

    fastboot boot out-umi/umi-boot.img

1.5 MB, RAM only. The loader then reports over USB as `xnuloader` (VID:PID `18D1:D00D`), and `xl`
takes it from there.

**On Windows, fastboot needs a driver.** With the phone in the bootloader, `fastboot devices` prints
nothing and Device Manager shows `USB\VID_18D1&PID_D00D` in an error state. MiFlash ships the right
INF at `Source/ThirdParty/Google/Driver/android_winusb.inf`; install it elevated:

    pnputil /add-driver "<path>\android_winusb.inf" /install

## Not done yet

- **The screen is unverified.** `FB_BASE=0x9C000000` follows from the map, and the map now comes from
  a Xiaomi SM8250 device rather than a OnePlus one, but whether alioth's display driver drives umi's
  panel is still a question only the phone answers. If the boot is black, the loader's own
  `display 1080 x 2340 at 0x...` line gives the real base to put back into `platform_umi`.
- **If alioth's drivers are still not enough**, the next step is umi's own: extract `xbl_4` / `xbl_5`
  with [UEFIReader](https://github.com/WOA-Project/UEFIReader) and put the result in `Binaries/umi`.
  The phone has no root (`ro.debuggable=0`), so that means either a Mi 10 fastboot ROM (~5 GB) or
  booting a TWRP image to get a root shell and `dd` the partition.
- **No touch driver.** umi is an ST Focaltech part on GENI I2C SE13 (`3-0049`); the kernel in this
  tree has redwood's Goodix GT9916 over bit-banged SPI. `BOOTARGS` in `umi.conf` therefore leaves
  `touchscreen=1` out — framebuffer plus CPU is enough for a lock screen.
- **No firmware.** No GPU, Wi-Fi, audio or haptics blobs are fetched, so SpringBoard gets software
  rendering only.
