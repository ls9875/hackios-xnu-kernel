/*
 * umi: paste into xnuloader/loader.c next to platform_redwood and add &platform_umi to platforms[].
 * Every value is explained in docs/PORTING.md ("The loader").
 *
 * Read from the phone's own device tree (boot.img dtb set, blob 0 -- see PORT-UMI.md), not guessed:
 *   /soc/interrupt-controller@17a00000  compatible "arm,gic-v3"
 *       reg = <0x17a00000 0x10000 0x17a60000 0x100000>   -> dist 0x17A00000, redist 0x17A60000
 *   /cpus/cpu@0,100,200,300  capacity-dmips-mhz = 1024   (Cortex-A55)
 *   /cpus/cpu@400,500,600,700 capacity-dmips-mhz = 1894  (Cortex-A77)
 *       -> 8 cores, MPIDR numbers them in the second affinity field (shift 8), small = 0x0f
 *
 * That is byte for byte the same GIC layout and the same cluster split as redwood (SM7325), which is
 * why only .name and .fb_base differ from platform_redwood.
 */
static const struct platform platform_umi = {
	/* io_base must be non-zero: XNU treats a zero SoC base as "no SoC" and skips clock setup. */
	.name = "umi", .io_base = 0x00400000, .io_size = 0x3FC00000,
	.gic_dist = 0x17A00000, .gic_redist = 0x17A60000, .uart = 0, .log_other_cpu = 1,
	.cores = 8, .core_shift = 8,
	/*
	 * Cores 0-3 are Cortex-A55s, 4-7 Cortex-A77s (2.42 GHz): the big cluster is rated 1894 against
	 * the little one's 1024 in the device tree.
	 */
	.small_cores = 0x0f,
	/*
	 * The base of the "Display Reserved" region in the memory map the umi UEFI installs (see
	 * uefi/files/Platforms/Xiaomi/umiPkg/Library/MemoryMapLib/MemoryMapLib.c), which is what its GOP
	 * reports -- the same relationship redwood's 0xE1000000 has to its own map. Not yet confirmed on
	 * hardware: if the panel stays dark, the loader's first lines say what the firmware really
	 * reported ("display 1080 x 2340 at ...") and this is the value to put here.
	 */
	.fb_base = 0x9C000000,
};
