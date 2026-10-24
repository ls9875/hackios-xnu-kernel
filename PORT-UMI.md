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

## What the phone actually does

Recorded on the real device, 24 October, after the port was rebased on alioth.

**The UEFI comes up and the loader reports over USB.** This is the milestone the port was missing
for two attempts:

    == XNULOADER v3 ==
    CurrentEL=1 MIDR=51df805e MMFR0=101122 CNTFRQ=19200000 image=a1000000
    display 1080 x 2340 at 9c000000
    platform umi
    usb: started

`display 1080 x 2340 at 9c000000` settles two things at once: `FB_BASE=0x9C000000` was right (it was
derived from the memory map, not measured), and the panel is driven at umi's real 2340, not alioth's
2400. `platform umi` means the framebuffer base matched, so `platform_umi` was picked. A screenshot
taken through the loader shows the Project Silicium splash underneath that text, so the display is
not just initialised, it is drawing.

**The kernel boots into user space.** With `root.apfs` as the root it gets all the way here:

    gic: v3, distributor ..., redistributor ..., virtual timer on PPI 27
    vm_page_bootstrap: 387079 free pages, 6137 wired pages
    MiniIOMobileFramebuffer: display 1125x2436 on a panel of 1080x2340 (4320 bytes per row)
    BSD root: md0, major 1, minor 0
    apfs: volume 0 "redwood" ...: 60 files, 17 directories
    bsd_init: done
    load_init_program: attempting to load /sbin/launchd

and then panics with `Process 1 exec of /sbin/launchd failed, errno 2`. That is expected and is not
a porting fault: `root.apfs` is the 4 MB image that belongs on `/redwood`, and it becomes the root
only when no system image is resident. `xl ramdisk userland/system.apfs` (2.5 GB, 26 MB/s over this
cable) is what makes the big image the root.

**SpringBoard runs, and the screen is the iOS home screen.** With the system image resident and
`bootcpu=7`:

    ./xl ramdisk userland/system.apfs          # 2376 MB, 26 MB/s over USB, 89 s
    XL_SECS=120 XL_CMDLINE="bootcpu=7" ./xl run

launchd comes up, spawns the real daemons, and SpringBoard draws. A screenshot taken through the
loader shows the iOS 16.5 home screen: status bar with the clock and battery, the Weather and
Calendar widgets, the app grid, and the dock. `ps` at the end of the run lists SpringBoard,
backboardd, logd, usermanagerd, fseventsd and containermanagerd.

**`bootcpu=7` is what keeps the phone alive.** The first attempt with the resident image passed
`cpus=8` without it. The kernel then started on the core the firmware hands over — which is the core
the loader watches from — took every core, and the phone hung with nothing on the wire:
`fastboot devices` printed `????????????` and every `getvar` timed out. No watchdog was armed (see
below), so it had to be power-cycled by hand. `cpus=1` without `bootcpu=7` hung the same way, so the
fault was never the core count: it is that the loader must not be left without a core. `bootcpu=7`
with `cpus` left at its default gives it one, and every run since has returned cleanly (`returned
462892 0`).

`wdt_arm()` used to return early for anything that was not redwood; it now returns early only for
`platform_qemu_virt`. That is right, but it does not help umi yet: every SMC returns
`0xffffffffffffffff`, i.e. this firmware does not implement Qualcomm's Haven virtual watchdog
(`VIRT_WDT_* 0x86000005..0008`), so `wdt_armed` ends up 0 either way. On this phone a hung kernel is
a phone that needs the power button — which is why `bootcpu=7` matters more here than it does on
redwood, where the same watchdog would have caught the hang.

## Not done yet

- **Touch, buttons, battery, GPU.** The kernel's device code is still redwood's. The log names the
  gaps plainly: `redwood: battery: SOC=-1% battery=0` (PMIC not driven), `redwood: input: the event
  server asks for property "..." which we do not have` (the HID service is incomplete and there is
  no touch driver at all, so the home screen cannot be poked), `redwood: SpringBoard: no GPU draws,
  vibrant views are plain` (software rendering). The Weather and Calendar widgets are blank for the
  same reason plus no network.
- **No touch driver.** umi is an ST Focaltech part on GENI I2C SE13 (`3-0049`); the kernel in this
  tree has redwood's Goodix GT9916 over bit-banged SPI. `BOOTARGS` in `umi.conf` therefore leaves
  `touchscreen=1` out. `docs/PORTING.md` §3 lists what the other redwood-specific kernel drivers
  (buttons, CPU frequency, watchdog, GPU, Wi-Fi, audio) each need to become umi's.
- **No firmware.** No GPU, Wi-Fi, audio or haptics blobs are fetched, so there is software rendering
  only and no network.
- **To reproduce what works now**, on a phone in Android or fastboot:

      fastboot boot out-umi/umi-boot.img        # 1.5 MB, RAM only
      ./xl ramdisk userland/system.apfs         # once per power cycle
      XL_SECS=120 XL_CMDLINE="bootcpu=7" ./xl run

`bootcpu=7` in any `xl run` is not optional — see above.
- **If a future symptom looks like the UEFI not starting at all**, the last resort is umi's own
  drivers: extract `xbl_4` / `xbl_5` with
  [UEFIReader](https://github.com/WOA-Project/UEFIReader) and put the result in `Binaries/umi`. The
  phone has no root (`ro.debuggable=0`), so that means either a Mi 10 fastboot ROM (~5 GB) or booting
  a TWRP image to get a root shell and `dd` the partition.
