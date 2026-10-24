#include <Library/MemoryMapLib.h>

/*
 * Xiaomi Mi 10 (umi, SM8250 "kona") memory map.
 *
 * Derived from Platforms/OnePlus/kebabPkg/Library/MemoryMapLib/MemoryMapLib.c (OnePlus 8T), because
 * that is the only SM8250 device in Mu-Silicium. Three things were checked against umi's own device
 * tree (the dtb set in its boot.img, see PORT-UMI.md) and one had to be changed:
 *
 *   kebab                       umi's /reserved-memory                      verdict
 *   HYP 0x80000000+0x00600000   hyp_region@80000000 +0x600000              same
 *   AOP CMD DB 0x80860000       reserved-memory@80860000 +0x20000          same
 *   SMEM 0x80900000+0x00200000  smem_region@80900000 +0x200000             same
 *   TZApps 0x82400000+0x09600000 removed_region@80b00000 +0x5300000        covered
 *   PIL 0x8BF00000+0x06500000   pil_* / cdsp_secure_heap, ends 0x92700000  covered (0x92400000)
 *   Display 0x9C000000+0x02400000 cont_splash@9c000000 + dfps@9e300000     same, ends 0x9E400000
 *   UEFI FD 0x9FC00000+0x00300000 (nothing -- free DDR above the splash)   same
 *
 * The change: kebab reserves {"Kernel", 0xA0000000, 0x10000000} and then puts its DXE heap at
 * 0xB0000000 with a length of 0x04500000, which reaches 0xF5000000. 0xF0000000 is PAYLOAD_BASE --
 * the address the boot shim copies the XNU container to and BootManagerLib starts it from (both
 * hardcoded: xnuloader/loader.c and SiliciumPkg/Library/BootManagerLib/XnuLoader.h). A UEFI built
 * with kebab's heap would therefore allocate pages on top of the kernel before it ever ran.
 *
 * So the heap is cut to 256 MB (0xB0000000..0xC0000000) and 0xF0000000+0x04000000 is reserved for the
 * payload explicitly, which is the shape redwood uses too ({"XNU Payload", 0xF0000000, 0x04000000}).
 *
 * Known and deliberate: PIL Reserved stops at 0x92400000 while umi's cdsp_secure_heap runs to
 * 0x92700000, so 3 MB of CDSP memory is not marked reserved here. Nothing in the UEFI allocates from
 * a region that is not listed, so that only means the DSP keeps its memory; it is left as kebab has it
 * rather than inventing a boundary the phone never told us.
 */
STATIC
EFI_MEMORY_REGION_DESCRIPTOR
gMemoryDescriptor[] = {
  // Name, Address, Length, HobOption, ResourceType, ResourceAttribute, MemoryType, ArmAttribute

  // DDR Regions
  {"HYP",              0x80000000, 0x00600000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, NS_DEVICE},
  {"MPSS EFS",         0x80600000, 0x00100000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, WRITE_BACK_XN},
  {"AOP CMD DB",       0x80860000, 0x00020000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"SMEM",             0x80900000, 0x00200000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"TZApps Reserved",  0x82400000, 0x09600000, HobOnlyNoCacheSetting, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"PIL Reserved",     0x8BF00000, 0x06500000, AddMem, MEM_RES, UNCACHEABLE, Reserv, UNCACHED_UNBUFFERED_XN},
  {"Sched Heap",       0x9BC00000, 0x00400000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  // The panel: the UEFI's GOP reports this address as its framebuffer base, and xnuloader picks
  // platform_umi by it (FB_BASE in common/devices/umi.conf). Same relationship as redwood's
  // 0xE1000000 "Display Reserved" == its fb_base.
  {"Display Reserved", 0x9C000000, 0x02400000, AddMem, MEM_RES, SYS_MEM_CAP, Reserv, WRITE_THROUGH_XN},
  {"DBI Dump",         0x9E400000, 0x00F00000, NoHob,  MMAP_IO, INITIALIZED, Conv,   UNCACHED_UNBUFFERED_XN},
  {"FV Region",        0x9F800000, 0x00200000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"ABOOT FV",         0x9FA00000, 0x00200000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, WRITE_BACK_XN},
  // FD_BASE/FD_SIZE in Resources/Configs/umi.toml and common/devices/umi.conf have to agree with this.
  {"UEFI FD",          0x9FC00000, 0x00300000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK},
  {"SEC Heap",         0x9FF00000, 0x0008C000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"MMU PageTables",   0x9FF8D000, 0x00003000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"UEFI Stack",       0x9FF90000, 0x00040000, AddMem, SYS_MEM, SYS_MEM_CAP, BsData, WRITE_BACK_XN},
  {"Log Buffer",       0x9FFF7000, 0x00008000, AddMem, SYS_MEM, SYS_MEM_CAP, RtData, WRITE_BACK_XN},
  {"Info Blk",         0x9FFFF000, 0x00001000, AddMem, SYS_MEM, SYS_MEM_CAP, RtData, WRITE_BACK_XN},
  // Where the ABL leaves the boot image (kebab's value, kept: same SoC, same boot chain).
  {"Kernel",           0xA0000000, 0x10000000, AddMem, MEM_RES, SYS_MEM_CAP, Reserv, WRITE_BACK_XN},
  // 256 MB instead of kebab's 0x04500000, so that the heap cannot reach PAYLOAD_BASE at 0xF0000000.
  {"DXE Heap",         0xB0000000, 0x10000000, AddMem, SYS_MEM, SYS_MEM_CAP, Conv,   WRITE_BACK_XN},
  // PAYLOAD_BASE, and PAYLOAD_MAX = 0x04000000 (mkxnuimg.py, xnuloader/loader.c).
  {"XNU Payload",      0xF0000000, 0x04000000, AddMem, SYS_MEM, SYS_MEM_CAP, Reserv, WRITE_BACK_XN},

  // Other Memory Regions
  {"IMEM Base",        0x14680000, 0x00040000, NoHob,  MMAP_IO, INITIALIZED, Conv,   NS_DEVICE},
  {"IMEM Cookie Base", 0x146BF000, 0x00001000, AddDev, MMAP_IO, INITIALIZED, Conv,   NS_DEVICE},

  // Register Regions
  {"IPC_ROUTER_TOP",   0x00400000, 0x00100000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"SECURITY CONTROL", 0x00780000, 0x00007000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_2_GSI",      0x00800000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_0_GSI",      0x00900000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"QUPV3_1_GSI",      0x00A00000, 0x000D0000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
  {"PRNG_CFG_PRNG",    0x00790000, 0x00010000, AddDev, MMAP_IO, UNCACHEABLE, MmIO,   NS_DEVICE},
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
