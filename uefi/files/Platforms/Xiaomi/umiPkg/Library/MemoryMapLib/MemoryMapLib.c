#include <Library/MemoryMapLib.h>

/*
 * Xiaomi Mi 10 (umi, SM8250 "kona") memory map.
 *
 * This is Platforms/Xiaomi/aliothPkg/Library/MemoryMapLib/MemoryMapLib.c (POCO F3 / Redmi K40,
 * SM8250-AC = Snapdragon 870) with one entry added. alioth is the same silicon from the same vendor,
 * and its DDR reservations match umi's own device tree -- not approximately, region for region:
 *
 *   alioth map                                umi's /reserved-memory                     verdict
 *   HYP              0x80000000+0x00600000    hyp_region@80000000 +0x600000              same
 *   AOP CMD DB       0x80860000+0x00020000    reserved-memory@80860000 +0x20000          same
 *   SMEM             0x80900000+0x00200000    smem_region@80900000 +0x200000             same
 *   QTEE + TZApps    0x80B00000+0x05300000    removed_region@80b00000 +0x5300000         same, exactly
 *   PIL Reserved     0x86200000+0x0C500000    pil_adsp/camera/cdsp/cvp/gpu/npu/...       same, ends 0x92700000
 *   Display Reserved 0x9C000000+0x02400000    cont_splash@9c000000 + dfps@9e300000       same, ends 0x9E400000
 *   UEFI FD          0x9FC00000+0x00300000    (free DDR above the splash)                free, see below
 *   UEFI Stack       0x9FF90000+0x00040000    (nothing)                                  PcdCPUCoresStackBase
 *
 * The earlier version of this port used kebab's (OnePlus 8T) map instead. Two things were wrong with
 * it. It reserved {"Kernel", 0xA0000000, 0x10000000} and put its DXE heap at 0xB0000000 with a length
 * of 0x04500000, reaching 0xF5000000 -- over PAYLOAD_BASE. And, more fundamentally, kebab's map only
 * agreed with umi's device tree in the display region, while alioth's agrees everywhere.
 *
 * The one entry added here is {"XNU Payload", 0xF0000000, 0x04000000}. 0xF0000000 is where the boot
 * shim copies the "XNUP" container and where BootManagerLib starts it from (both hardcoded:
 * xnuloader/loader.c and SiliciumPkg/Library/BootManagerLib/XnuLoader.h). It is not in alioth's map at
 * all, so nothing there would touch it -- but "nothing would touch it" is the kind of assumption that
 * costs a day when it turns out to be wrong, and redwood reserves it explicitly for the same reason.
 * PAYLOAD_MAX is 0x04000000 (mkxnuimg.py asserts it), hence the length.
 *
 * "Display Reserved" is also FB_BASE: the UEFI's GOP reports this address as its framebuffer base and
 * xnuloader picks platform_umi by it (common/devices/umi.conf). Same relationship as redwood's
 * 0xE1000000. "UEFI FD" has to stay in step with FD_BASE/FD_SIZE in Resources/Configs/umi.toml and
 * common/devices/umi.conf, and "UEFI Stack" with PcdCPUCoresStackBase in umi.dsc.
 */
STATIC
EFI_MEMORY_REGION_DESCRIPTOR
gMemoryDescriptor[] = {
  // Name, Address, Length, HobOption, ResourceType, ResourceAttribute, MemoryType, ArmAttribute

  // DDR Regions
  {"HYP",              0x80000000, 0x00600000, AddMem, MEM_RES, SYS_MEM_CAP, Reserv, NS_DEVICE},
  {"AOP CMD DB",       0x80860000, 0x00020000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"GPU PRR",          0x80880000, 0x00010000, AddMem, MEM_RES, WRITE_COMBINEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"SMEM",             0x80900000, 0x00200000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"QTEE",             0x80B00000, 0x01900000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"TZApps",           0x82400000, 0x03A00000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"PIL Reserved",     0x86200000, 0x0C500000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"ADSP RPC",         0x92700000, 0x00800000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  // Ends at 0x9BC00000: below the panel, and nowhere near PAYLOAD_BASE.
  {"DXE Heap",         0x98900000, 0x03300000, AddMem, SYS_MEM, SYS_MEM_CAP, Conv,   WRITE_BACK_XN},
  // The panel. FB_BASE in common/devices/umi.conf and platform_umi.fb_base in xnuloader/loader.c.
  {"Display Reserved", 0x9C000000, 0x02400000, AddMem, MEM_RES, SYS_MEM_CAP, Reserv, WRITE_THROUGH_XN},
  // FD_BASE/FD_SIZE in Resources/Configs/umi.toml and common/devices/umi.conf have to agree with this.
  {"UEFI FD",          0x9FC00000, 0x00300000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK},
  // PcdCPUCoresStackBase/PcdCPUCorePrimaryStackSize in umi.dsc.
  {"UEFI Stack",       0x9FF90000, 0x00040000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"Info Blk",         0x9FFFF000, 0x00001000, AddMem, SYS_MEM, SYS_MEM_CAP, RtData, WRITE_BACK_XN},
  // PAYLOAD_BASE, PAYLOAD_MAX = 0x04000000 (mkxnuimg.py, xnuloader/loader.c). The one entry that is
  // this port's own addition -- see the comment above.
  {"XNU Payload",      0xF0000000, 0x04000000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, WRITE_BACK_XN},

  // Other Memory Regions
  {"IMEM Base",        0x14680000, 0x00040000, NoHob,  MMAP_IO, INITIALIZED, Conv,   NS_DEVICE},
  {"IMEM Cookie Base", 0x146BF000, 0x00001000, AddDev, MMAP_IO, INITIALIZED, Conv,   NS_DEVICE},

  // Register Regions
  {"IPC_ROUTER_TOP",   0x00400000, 0x00100000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SECURITY CONTROL", 0x00780000, 0x00007000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PRNG_CFG_PRNG",    0x00790000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_2_GSI",      0x00800000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_0_GSI",      0x00900000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_1_GSI",      0x00A00000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"CRYPTO0 CRYPTO",   0x01DC0000, 0x00040000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TCSR_TCSR_REGS",   0x01FC0000, 0x00030000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GPU_GMU_CX_BLK",   0x02C7D000, 0x00002000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"GPU_CC",           0x02C90000, 0x0000A000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_SSC_GSI",    0x05A00000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PERIPH_SS",        0x08800000, 0x00200000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"USB30_PRIM",       0x0A600000, 0x0011B000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"USB_RUMI",         0x0A720000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"USB30_SEC",        0x0A800000, 0x0011B000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"AOSS",             0x0B000000, 0x04000000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_WEST",        0x0F100000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_SOUTH",       0x0F500000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"TLMM_NORTH",       0x0F900000, 0x00300000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SMMU",             0x15000000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"APSS_HM",          0x17800000, 0x0d981000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE}
};

VOID
GetMemoryMap (
  OUT EFI_MEMORY_REGION_DESCRIPTOR **MemoryDescriptor,
  OUT UINT8                         *MemoryDescriptorCount
  )
{
  // Pass Data
  *MemoryDescriptor      = gMemoryDescriptor;
  *MemoryDescriptorCount = ARRAY_SIZE (gMemoryDescriptor);
}
