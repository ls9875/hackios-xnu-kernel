/*
 * The touch screen of the Xiaomi Mi 10 (umi): an STMicroelectronics FingerTipS controller on the
 * SoC's I2C bus 3 at address 0x49, as a source of touches for /dev/redwood-touch
 * (sys/redwood_input.h). Only with boot-arg umi_touch=1.
 *
 * "fts" does not mean FocalTech here. The node is compatible = "st,fts" with
 * fts,default-fw-name = "st_fts_j2.ftb", and the driver Android runs for it is
 * drivers/input/touchscreen/st/ in Xiaomi's own kernel source (MiCode/Xiaomi_Kernel_OpenSource,
 * branch umi-q-oss). That driver is the reference for everything in this file; this comment is
 * not.
 *
 * Where every number comes from -- umi.dtb, the phone's own device tree, plus the dtbo
 * overlays the boot loader applies to it. The one that matters is overlay-17.dtbo entry 2,
 * the only entry carrying "st,fts" (the others carry "focaltech,fts" over SPI, which is a
 * different panel variant; Android's /dev/input/event5 is called "fts", so this is the live
 * one):
 *
 *   bus      qupv3_se13_i2c, which in umi.dtb is soc/i2c@a94000 (compatible "qcom,i2c-geni",
 *            reg 0xa94000, size 0x4000). The overlay's fragment@111 targets that label and
 *            hangs fts@49 off it, which is where the 0x49 -- and the "3-0049" the phone
 *            shows under /sys/bus/i2c/devices -- come from. The bus's pinctrl-0 is
 *            qupv3_se13_i2c_active, gpio36 and gpio37 in function "qup13".
 *   pins     pmx_ts_active muxes gpio38 and gpio39 as plain GPIOs instead:
 *            fts,reset-gpio = <gpio38 0>, fts,irq-gpio = <gpio39 0x2008>.
 *   power    avdd-supply -> touch_vddio_vreg ("disp_vddio_vreg"), a regulator-fixed whose
 *            enable is gpio69 and which is active high. It is the panel's I/O rail: on
 *            already, because the display is lit. The driver leaves it alone unless asked
 *            (umi_touch_avdd=1 reads it and drives it high) rather than assuming.
 *   panel    fts,x-max 1080, fts,y-max 2340, focaltech,max-touch-number 10.
 *
 * The serial engine itself is not used. Its I2C protocol firmware is loaded by whoever
 * configures engine 13 for I2C, and nothing on this boot path does -- the loader brings up
 * the display and USB and nothing else -- so there is no firmware in the engine to talk to.
 * Instead gpio36 and gpio37 are reconfigured as GPIOs and I2C is driven by hand: open
 * drain, one pin per line, a few microseconds per half clock. I2C wants no clock, no DMA
 * and no engine, and the register protocol is plain once it is the right one -- a command
 * byte 0xB6, a 16-bit address, and one dummy byte in front of the data on a read. See the
 * protocol section below; getting that wrong is what made this chip look dead for a day.
 *
 * Diagnostics, because the first run is the one that has to say what the chip is:
 *   umi_touch_diag=1    after the chip ID, read hardware registers 0x0000..0x00ff and print
 *                       what came back: the chip's map, in the loader's log
 *   umi_touch_swap=1    start with the data and clock pins the other way round (the driver
 *                       tries both anyway; nothing is written down anywhere about which of
 *                       gpio36/gpio37 is SDA and which is SCL)
 *   umi_touch_poll_us=N look at the interrupt line every N microseconds (8000 otherwise)
 *   umi_touch_flip=N    1 mirror left-right, 2 top-bottom, 3 both
 *   umi_touch_avdd=1    read the panel I/O rail's enable (gpio69) and drive it high before
 *                       anything else
 *   umi_touch_stage=N   stop early and say so: 1 read the pins and nothing else, 2 also reset
 *                       the chip and read its ID, 9 (the default) run the polling loop
 *   umi_touch_gpio=0xN  put every pin at N + 0x1000 * pin, over the tile table below
 *   umi_touch_pushpull=1  drive the high half of the clock rather than waiting for a pull-up,
 *                       for a board whose pull-ups are not powered in this boot environment
 *   umi_touch_cmd_db=1  print what RPMh can switch, with the address of each, and stop
 *   umi_touch_scan=1    ask every address on the bus who is there, before anything else, and stop
 *   umi_touch_supply=1  put the board back the way Android leaves it before touching the bus:
 *                       smpa4 on (undoing an earlier pcycle run), ldoc1 on -- the 1.8 V this
 *                       pair's pull-ups hang from, proved by run 152817 -- gpio69, the switch
 *                       the fts node names, then the three rails below the display tree
 *                       (gpio5 vdd_boost_vreg, gpio71 display_panel_vci, gpio12 ext_5v_boost),
 *                       then a reset pulse, reading the chip id after each step
 *   umi_touch_proto=1   the chip acknowledges its address and returns zero for every register:
 *                       read the ID through sixteen opcodes, dump the whole 256-byte map, and
 *                       write/read-back sixteen addresses, to tell a wrong opcode from a core
 *                       that is not running. Powers up first and stops after.
 *   umi_touch_pcycle=1  switch smpa4 off and back on with the reset held low, then send the
 *                       rom's warm boot: the only thing that clears state a previous run left
 *                       behind, because `fastboot boot` never power-cycles the SoC. Stops after.
 *                       It does not put smpa4 back -- umi_touch_supply=1 does, and every run
 *                       after 214827 opened with the bus clamped because nothing did.
 *   umi_touch_diff=1    read the whole map three times and print only what changes between
 *   umi_touch_wtest=1   read the ID and the firmware version five times, then stop: a stable
 *                       pair proves the dummy byte is being dropped in the right place
 *   umi_touch_settle_ms=N  how long the chip gets between the reset and the first read (200)
 *   umi_touch_sweep=1   try both ID/firmware address pairs at both bus addresses, one line each
 *   umi_touch_reset=N   1 (the default) leaves the reset line high after the pulse, 0 leaves it
 *                       low, 2 does not touch it
 *   umi_touch_bit_us=N  a half clock in microseconds (3 by default, about 150 kHz)
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/redwood_input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

/* ---- pins (TLMM) ------------------------------------------------------------------------------- */

/*
 * The GPIO registers are not where soc/pinctrl@f000000 says they are. That node is
 * reg = <0x0f000000 0x1000000>, one window over three separate register tiles, which this
 * port's own platform memory map names (uefi/files/Platforms/Xiaomi/umiPkg/Library/
 * MemoryMapLib/MemoryMapLib.c):
 *
 *   TLMM_WEST   0x0f100000  0x300000
 *   TLMM_SOUTH  0x0f500000  0x300000
 *   TLMM_NORTH  0x0f900000  0x300000
 *
 * A pin's control register is its tile's base plus 0x1000 per pin number, and which tile a
 * pin is in is not "the low ones west, the high ones north" -- it interleaves. The table in
 * gpio_tile() below is not a guess: it was read out of the pin table in the phone's own
 * Android kernel, a 180-entry array of {ctl, io, intr_cfg, intr_status, intr_target} at an
 * 80-byte stride, every entry satisfying io == ctl + 4, intr_cfg == ctl + 8,
 * intr_status == ctl + 0xc, with ctl == tile + 0x1000 * pin.
 *
 * Getting it wrong is not a wrong reading, it is a dead phone: the computed address lands in
 * a hole the tile does not decode, the access raises a synchronous external abort, and the
 * machine goes down before a single printf reaches the loader's log. That is what
 * 0x0f100000 + 0x1000 * 36 = 0x0f124000 did -- TLMM_WEST's first implemented pin is
 * 0x0f185000 (gpio133) -- and why every earlier attempt at this driver, and
 * redwood_goodix.c, whose pins 52 to 55 and 105 to 107 land in the same hole, hung with
 * nothing on the wire. The POCO X5 Pro that driver was written for (yupik) has one tile at
 * 0x0f100000 and never meets this.
 *
 * All five pins the bus and the chip itself use -- 36, 37, 38, 39 and 69 -- are in TLMM_SOUTH:
 * gpio36 at 0x0f524000, gpio39 at 0x0f527000, gpio69 at 0x0f545000. The two rails below the
 * display tree are not: gpio5 and gpio12 are in TLMM_NORTH, gpio71 is in TLMM_SOUTH again.
 */
#define TLMM_WEST                0x0f100000UL
#define TLMM_SOUTH               0x0f500000UL
#define TLMM_NORTH               0x0f900000UL
#define TLMM_SIZE                0x100000UL

#define GPIO_CFG(n)              (gpio_tile(n) + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)               (gpio_tile(n) + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define PULL_NONE                0
#define PULL_DOWN                1
#define PULL_UP                  3

#define PIN_SDA                  36      /* qupv3_se13_i2c_active: "qup13" */
#define PIN_SCL                  37      /* qupv3_se13_i2c_active: "qup13" */
#define PIN_RESET                38      /* fts,reset-gpio; low = in reset */
#define PIN_INT                  39      /* fts,irq-gpio; low = the chip has a report */
#define PIN_AVDD                 69      /* touch_vddio_vreg ("disp_vddio_vreg"), active high */
/*
 * The other three rails in the same fragment of the panel overlay. touch_vddio_vreg is the one the
 * fts node names -- avdd-supply points at it -- but a part that acknowledges its address and
 * returns zero for all 256 registers, refuses writes and ignores sense-on is a part whose i/o ring
 * is powered and whose core is not, and the i/o ring is not the only thing on this board that has
 * to be switched on. All three of these are in /fragment@76 of fts-entry.dtb:
 *
 *   vdd_boost_vreg       gpio5    regulator-always-on, 4 ms startup delay
 *   display_panel_vci    gpio71   3.0 V, regulator-boot-on
 *   ext_5v_boost         gpio12   5 V, regulator-enable-ramp-delay 1600
 *
 * "always-on" is the one that matters: Linux enables a regulator-always-on node when it registers
 * it, without anything asking for it, so Android has had gpio5 high since boot and this kernel
 * never has. A touch asic's avdd is typically 2.8-3.3 V and comes off the display's power tree,
 * which is what these three are.
 */
#define PIN_VBOOST               5       /* vdd_boost_vreg */
#define PIN_V5                   12      /* ext_5v_boost */
#define PIN_VCI                  71      /* display_panel_vci */

#define I2C_ADDRESS              0x49    /* fts@49 */

/* ---- the protocol: STMicroelectronics FingerTipS, not FocalTech ------------------------------- */

/*
 * "fts" here does not mean FocalTech. umi's node is compatible = "st,fts" with
 * fts,default-fw-name = "st_fts_j2.ftb", and the driver Android runs for it is
 * drivers/input/touchscreen/st/ in Xiaomi's own kernel source (branch umi-q-oss of
 * MiCode/Xiaomi_Kernel_OpenSource). Every number below is taken from that driver, and it is a
 * different protocol from the one this file was written to.
 *
 * There is no linear register file. An access is a command carrying a *16-bit* address, and a read
 * answers with one dummy byte in front of the data:
 *
 *   read    [0xB6, addr_hi, addr_lo]  + repeated start + (1 dummy + N) bytes   ftsIO.c:readCmdU16
 *   write   [0xB6, addr_hi, addr_lo, data ...]  in one transaction             ftsIO.c:writeCmdU16
 *
 * Reading 0x00..0xFF with an 8-bit address -- what this driver used to do -- cannot work: the chip
 * takes the register byte as the high half of an address, waits for a low half that never comes,
 * and answers nothing. That is exactly the map it produced: 252 registers reading 0x00, writes
 * acknowledged and not sticking, and a chip ID of 00 00. It was not a dead chip.
 */
#define FTS_CMD_HW_REG           0xb6    /* FTS_CMD_HW_REG_R and _W are the same opcode */
#define FTS_CMD_READ_ONE         0x85    /* FIFO_CMD_READONE: one event, FTS_EVENT_SIZE bytes */
#define FTS_CMD_SENSE_ON         0x93    /* FTS_CMD_MS_MT_SENSE_ON: start scanning the panel */
#define FTS_IER_ADDR             0x001c  /* IER_ADDR */
#define FTS_IER_ENABLE           0x41    /* IER_ENABLE: let the chip drive the interrupt line */
#define FTS_EVENT_SIZE           8       /* FIFO_EVENT_SIZE */
#define FTS_FIFO_DEPTH           32      /* FIFO_DEPTH */

/*
 * FTM3_CHIP is a compile-time switch in the vendor driver and it moves the ID, the firmware
 * version and the reset register. Nothing here says which one umi is, so both are tried: the set
 * that answers is the set this phone has.
 */
#define FTS_CHIP_ID_ADDR_A       0x0004  /* DCHIP_ID_ADDR, FTM3_CHIP undefined */
#define FTS_CHIP_ID_ADDR_B       0x0007  /* DCHIP_ID_ADDR, FTM3_CHIP defined */
#define FTS_FW_VER_ADDR_A        0x0008  /* DCHIP_FW_VER_ADDR, undefined */
#define FTS_FW_VER_ADDR_B        0x000a  /* defined */
#define FTS_SYS_RESET_ADDR_A     0x0028  /* SYSTEM_RESET_ADDRESS, undefined */
#define FTS_SYS_RESET_VAL_A      0x80    /* SYSTEM_RESET_VALUE */
#define FTS_SYS_RESET_ADDR_B     0x0023  /* defined */
#define FTS_SYS_RESET_VAL_B      0x01
#define FTS_FW_VER_BYTES         2       /* DCHIP_FW_VER_BYTE */

#define FTS_ID_0_A               0x36    /* DCHIP_ID_0, DCHIP_ID_1: undefined */
#define FTS_ID_1_A               0x70
#define FTS_ID_0_B               0x39    /* FTM3_CHIP defined */
#define FTS_ID_1_B               0x6c

/* Event ids, ftsSoftware.h. A pointer event is 8 bytes; see parse_event() for the layout. */
#define EVENTID_NO_EVENT         0x00
#define EVENTID_ENTER_POINTER    0x03
#define EVENTID_LEAVE_POINTER    0x04
#define EVENTID_MOTION_POINTER   0x05
#define EVENTID_ERROR_EVENT      0x0f
#define EVENTID_CONTROL_READY    0x10
#define EVENTID_STATUS_UPDATE    0x16
#define EVENTID_ECHO             0xec

#define FTS_MAX_TOUCHES          10      /* focaltech,max-touch-number, and ABS_MT_SLOT max 9 */

static vm_offset_t tlmm_west, tlmm_south, tlmm_north;
static vm_offset_t tlmm_one;            /* umi_touch_gpio: put every pin at this base instead */
static uint32_t stage = 9;              /* umi_touch_stage */
static uint32_t reset_mode = 1;         /* umi_touch_reset: the level the reset sequence settles at */
static uint32_t settle_ms = 200;        /* umi_touch_settle_ms: how long the chip gets after reset */

/* Which tile a pin's registers live in, out of the table described above. */
static vm_offset_t
gpio_tile(unsigned pin)
{
	if (tlmm_one) {
		return tlmm_one;
	}
	if (pin <= 3) {
		return tlmm_south;
	}
	if (pin <= 23) {
		return tlmm_north;
	}
	if (pin <= 27) {
		return tlmm_south;
	}
	if (pin <= 31) {
		return tlmm_north;
	}
	if (pin <= 65) {
		return tlmm_south;
	}
	if (pin <= 68) {
		return tlmm_north;
	}
	if (pin <= 76) {
		return tlmm_south;
	}
	if (pin <= 84) {
		return tlmm_north;
	}
	if (pin <= 91) {
		return tlmm_south;
	}
	if (pin <= 124) {
		return tlmm_north;
	}
	if (pin <= 132) {
		return tlmm_south;
	}
	return tlmm_west;               /* 133 .. 179 */
}

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

/*
 * Every wait in here spins. delay_for_interval() sleeps for anything but the shortest intervals,
 * and on this boot path a thread that sleeps does not come back: the run that first got as far
 * as the reset stopped dead at the 5 ms wait after driving gpio69, and the one before it stopped
 * inside the bus-recovery clock pulses. A bit-banged clock wants the line held for three
 * microseconds, not for a scheduler timeslice -- which would break I2C even if it returned --
 * and a reset sequence wants 200 ms of wall time whatever else is runnable. So wait on the
 * timebase, which redwood_adreno.c and redwood_apfs_check.c already do on this port.
 */
static void
sleep_ns(uint64_t ns)
{
	uint64_t deadline;

	clock_interval_to_deadline((uint32_t)ns, 1, &deadline);
	while (mach_absolute_time() < deadline) {
		/* spin */
	}
}

static void
sleep_us(uint32_t us)
{
	sleep_ns((uint64_t)us * NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	sleep_ns((uint64_t)ms * NSEC_PER_MSEC);
}

static void
gpio_configure(unsigned pin, unsigned function, unsigned pull, unsigned ma, bool output)
{
	wr(GPIO_CFG(pin), pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
gpio_set(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
}

static bool
gpio_get(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
gpio_output(unsigned pin, bool high, unsigned pull, unsigned ma)
{
	gpio_set(pin, high);            /* the level first, so that the pin never drives the wrong one */
	gpio_configure(pin, 0, pull, ma, true);
}

/* What the pins look like: said once at the start, so a run that goes nowhere still says why. */
static void
show_pins(const char *when)
{
	printf("umi_touch: %s: tlmm sda %#x/%u scl %#x/%u reset %#x/%u int %#x/%u\n", when,
	    rd(GPIO_CFG(PIN_SDA)), gpio_get(PIN_SDA), rd(GPIO_CFG(PIN_SCL)), gpio_get(PIN_SCL),
	    rd(GPIO_CFG(PIN_RESET)), gpio_get(PIN_RESET), rd(GPIO_CFG(PIN_INT)), gpio_get(PIN_INT));
}

/* ---- I2C by hand -------------------------------------------------------------------------------- */

static unsigned pin_sda = PIN_SDA, pin_scl = PIN_SCL;
static uint32_t bit_us = 3;         /* a half clock; about 150 kHz with 3 */

/*
 * umi_touch_pushpull=1 drives the high half of the clock instead of letting a pull-up do it.
 *
 * It is not how I2C is meant to work: the point of open drain is that the slave can pull a line
 * down while the master holds it up, and here both would drive. It is here because this pair has
 * no working pull-up in this boot environment -- released, with the pull field set to each of its
 * four values in turn, both lines sit at 0 within 200 us -- so the only high level the bit-bang
 * ever produces is one it drives itself. Contention is limited to the acknowledge bit of each
 * byte and to a 2 mA driver, which is the price of finding out whether the chip is there at all:
 * if it talks this way it is powered and the fault is the pull-ups; if it does not, nothing is
 * driving that pair and the chip has no supply.
 */
static bool pushpull;
/*
 * The old umi_touch_reg16 and umi_touch_sweep arguments that used to live here are gone with the
 * 8-bit register model they existed to test. The answer they were looking for is in the protocol
 * section above: the address is 16 bits, always, and it is not a variant to try.
 */
static uint8_t bus_address = I2C_ADDRESS;

/*
 * Where the bring-up is, one line per step. A run that stops leaves its last line behind, so
 * "no chip ID" stops being one uninformative silence and becomes a place -- which is the only
 * way to find anything from a log when the alternative is a phone that says nothing at all.
 */
static void
trace(const char *what)
{
	printf("umi_touch: ... %s: sda %#x/%u scl %#x/%u reset %#x/%u int %#x/%u\n", what,
	    rd(GPIO_CFG(pin_sda)), gpio_get(pin_sda), rd(GPIO_CFG(pin_scl)), gpio_get(pin_scl),
	    rd(GPIO_CFG(PIN_RESET)), gpio_get(PIN_RESET),
	    rd(GPIO_CFG(PIN_INT)), gpio_get(PIN_INT));
}

/* A line is open drain: released is an input with the pull-up on, pulled down is an output. */
static void
line_release(unsigned pin)
{
	if (pushpull) {
		gpio_output(pin, true, PULL_NONE, 2);
	} else {
		gpio_configure(pin, 0, PULL_UP, 2, false);
	}
}

static void
line_low(unsigned pin)
{
	gpio_output(pin, false, PULL_NONE, 2);
}

static void
i2c_delay(void)
{
	sleep_us(bit_us);
}

/*
 * Let SCL go and wait until it is really high. A slave is allowed to hold SCL down until it has
 * the byte ready -- clock stretching -- and this driver used to read through that: it released
 * the line, waited three microseconds and sampled SDA whether or not SCL had actually risen.
 * Writing the address byte survives that, because the slave has nothing to prepare; reading a
 * register does not, and a chip that stretches answers every read with zeros, which is exactly
 * the map this one gave: four non-zero bytes out of 256, unchanged at 5 kHz, on a bus that
 * acknowledges its address perfectly well.
 */
static void
scl_high(void)
{
	uint64_t deadline;

	line_release(pin_scl);
	clock_interval_to_deadline(4000, NSEC_PER_USEC, &deadline);     /* 4 ms of wall time, at most */
	while (!gpio_get(pin_scl) && mach_absolute_time() < deadline) {
		/* the slave is holding SCL down; that is it asking for time, so give it */
	}
}

static void
i2c_start(void)
{
	line_release(pin_sda);
	scl_high();
	i2c_delay();
	line_low(pin_sda);              /* SDA falls while SCL is high */
	i2c_delay();
	line_low(pin_scl);
	i2c_delay();
}

static void
i2c_stop(void)
{
	line_low(pin_sda);
	i2c_delay();
	scl_high();          /* SCL rises */
	i2c_delay();
	line_release(pin_sda);          /* SDA rises while SCL is high */
	i2c_delay();
}

static bool
i2c_write_byte(uint8_t value)
{
	for (int bit = 7; bit >= 0; bit--) {
		if (value >> bit & 1) {
			line_release(pin_sda);
		} else {
			line_low(pin_sda);
		}
		i2c_delay();
		scl_high();
		i2c_delay();
		line_low(pin_scl);
		i2c_delay();
	}
	/* the acknowledge: the chip holds SDA down for one clock */
	line_release(pin_sda);
	i2c_delay();
	scl_high();
	i2c_delay();
	bool ack = !gpio_get(pin_sda);
	line_low(pin_scl);
	i2c_delay();
	return ack;
}

static uint8_t
i2c_read_byte(bool ack)
{
	uint8_t value = 0;

	line_release(pin_sda);
	for (int bit = 7; bit >= 0; bit--) {
		i2c_delay();
		scl_high();
		i2c_delay();
		value = (uint8_t)(value << 1 | (gpio_get(pin_sda) ? 1 : 0));
		line_low(pin_scl);
		i2c_delay();
	}
	/* our acknowledge: SDA down for the bytes we want more of, up for the last one */
	if (ack) {
		line_low(pin_sda);
	}
	i2c_delay();
	scl_high();
	i2c_delay();
	line_low(pin_scl);
	line_release(pin_sda);
	i2c_delay();
	return value;
}

/*
 * A transfer left half finished holds a line down; nine clocks let the chip let go of it.
 *
 * Both lines are checked, not just SDA. A slave that is waiting for the rest of a byte stretches
 * the *clock*, and holding SCL down is the whole point of that: SDA can be high at the same time
 * and this function used to read that as "the bus is idle" and return without doing anything.
 * The state then survives, because `fastboot boot` does not power-cycle the SoC, so the next run
 * starts with the chip still mid-byte and every read it makes returns zeros with an acknowledge
 * on every byte -- which is indistinguishable from a chip that is answering zeros.
 *
 * That is not hypothetical: run 214959 opened with `sda 0x3/0 scl 0x3/0`, both lines low with the
 * pull-ups on, and one two-byte register read took 267 ms instead of 0.7 ms, because scl_high()
 * was timing out on every single clock. The runs before it had left the chip there.
 */
static void
i2c_bus_recover(void)
{
	int pulses;

	line_release(pin_sda);
	line_release(pin_scl);
	i2c_delay();
	if (gpio_get(pin_sda) && gpio_get(pin_scl)) {
		return;                     /* both lines high: the bus is idle */
	}
	printf("umi_touch: the bus is stuck (sda %u scl %u); clocking it free\n",
	    gpio_get(pin_sda), gpio_get(pin_scl));
	for (pulses = 0; pulses < 9 && !(gpio_get(pin_sda) && gpio_get(pin_scl)); pulses++) {
		line_low(pin_scl);
		i2c_delay();
		line_release(pin_scl);
		i2c_delay();
	}
	printf("umi_touch: ... %d pulse(s), sda %u scl %u\n", pulses,
	    gpio_get(pin_sda), gpio_get(pin_scl));
	line_low(pin_sda);
	i2c_delay();
	line_release(pin_scl);
	i2c_delay();
	line_release(pin_sda);
	i2c_delay();
	printf("umi_touch: ... recover done, sda %u scl %u\n", gpio_get(pin_sda), gpio_get(pin_scl));
}

/*
 * Ask every 7-bit address on the bus whether anyone is home: start, the address byte, look at the
 * acknowledge, stop. Nothing is written anywhere and no register is read, so a scan cannot disturb
 * whatever answers -- and it settles two questions at once. If nothing answers at any address, the
 * bit-bang is producing something no device recognises, or nothing on that pair is powered. If
 * something answers at an address other than 0x49, then the panel variant on this phone is not the
 * one fts@49 describes: the same overlay carries focaltech@38 with the same supply.
 */
static bool
i2c_address_present(uint8_t address)
{
	bool ack;

	i2c_start();
	ack = i2c_write_byte((uint8_t)(address << 1));
	i2c_stop();
	return ack;
}

static unsigned
scan_bus(void)
{
	unsigned found = 0;

	for (unsigned a = 0x08; a <= 0x77; a++) {
		if (i2c_address_present((uint8_t)a)) {
			printf("umi_touch:   %#04x acknowledges\n", a);
			found++;
		}
	}
	return found;
}

/* ---- the FingerTipS protocol on top of the bit-bang -------------------------------------------- */

/*
 * Read `length` bytes of the hardware register at `address`.
 *
 * ftsIO.c:readCmdU16 writes {0xB6, hi, lo}, repeats the start, and reads length + 1 bytes because
 * the chip puts one dummy byte in front of the data (DUMMY_HW_REG is 1 for every hardware register
 * in this part). Dropping that byte is the whole difference between the data and garbage shifted
 * by one, and it is not optional: it is how the vendor driver reads the firmware version.
 */
static bool
fts_read_registers(uint16_t address, uint8_t *data, uint32_t length)
{
	uint8_t buf[FTS_FIFO_DEPTH + 1];
	bool ok = false;

	if (length > sizeof(buf) - 1) {
		return false;
	}
	i2c_start();
	if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(FTS_CMD_HW_REG) &&
	    i2c_write_byte((uint8_t)(address >> 8)) &&
	    i2c_write_byte((uint8_t)(address & 0xff))) {
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
			for (uint32_t i = 0; i <= length; i++) {
				buf[i] = i2c_read_byte(i < length);
			}
			memcpy(data, buf + 1, length);
			ok = true;
		}
	}
	i2c_stop();
	return ok;
}

/*
 * The same read with an 8-bit register address: {0xB6, reg}, repeated start, dummy, data.
 *
 * That is what this driver did before the vendor source was read, and the sweep that replaced it
 * recorded one thing the 16-bit model cannot explain: an 8-bit walk of 0x00..0xFF found four
 * registers that were not zero, while every 16-bit walk of the same range finds none. Four is
 * either the tail of a real answer or four bytes of noise, and a count cannot tell the two apart.
 * This walks the range again and prints the addresses, which is the part the count threw away.
 */
static bool
fts_read_reg8(uint8_t reg, uint8_t *data, uint32_t length)
{
	uint8_t buf[FTS_FIFO_DEPTH + 1];
	bool ok = false;

	if (length > sizeof(buf) - 1) {
		return false;
	}
	i2c_start();
	if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(FTS_CMD_HW_REG) &&
	    i2c_write_byte(reg)) {
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
			for (uint32_t i = 0; i <= length; i++) {
				buf[i] = i2c_read_byte(i < length);
			}
			memcpy(data, buf + 1, length);
			ok = true;
		}
	}
	i2c_stop();
	return ok;
}

/*
 * Where in the address space, if anywhere, does this part answer with something other than zero?
 *
 * "The register file is not there" has been the reading of every run so far, and it rests on one
 * range: 0x0000..0x00ff. That is 256 of 65536 addresses, chosen because it is where the vendor
 * header puts the id. If the file is elsewhere -- a page register, a window at 0x2000 where the
 * command and the fifo live -- then the chip has been answering all along and the map was wrong,
 * which is a completely different problem from a core that is not running. So: eight-bit and
 * sixteen-bit walks of the id page, then the rest of the space a window at a time, and any byte
 * that is not zero is printed with the address it came from.
 */
static void
register_scan(void)
{
	static const uint16_t windows[] = { 0x0000, 0x0100, 0x0200, 0x2000, 0x2100, 0x3000, 0xc000, 0xe000 };
	uint8_t value;
	unsigned hits;

	printf("umi_touch: --- eight-bit walk of 0x00..0xff, non-zero addresses only ---\n");
	hits = 0;
	for (unsigned reg = 0; reg < 0x100; reg++) {
		value = 0;
		if (!fts_read_reg8((uint8_t)reg, &value, 1)) {
			continue;
		}
		if (value != 0) {
			printf("umi_touch:   8-bit %02x = %02x\n", reg, value);
			hits++;
		}
	}
	printf("umi_touch:   8-bit: %u of 256 not zero\n", hits);

	for (unsigned w = 0; w < sizeof(windows) / sizeof(windows[0]); w++) {
		hits = 0;
		for (unsigned off = 0; off < 0x100; off++) {
			value = 0;
			if (!fts_read_registers((uint16_t)(windows[w] + off), &value, 1)) {
				continue;
			}
			if (value != 0) {
				printf("umi_touch:   16-bit %04x = %02x\n", windows[w] + off, value);
				hits++;
			}
		}
		printf("umi_touch:   16-bit window %04x: %u of 256 not zero\n", windows[w], hits);
	}
}

/*
 * The same read with the opcode spelled out.
 *
 * Every other function in here assumes 0xB6 is the opcode that means "hardware register", because
 * ftsHardware.h says so and it is the same byte for reading and writing. That assumption is one of
 * the two things that can make a live chip look dead: a slave acknowledges *any* byte sent to its
 * address, so a wrong opcode is not a failure at the electrical level -- it is a command the chip
 * does not understand, answered with zeros. "Acknowledges its address and returns zeros to
 * everything" is exactly what this chip has done on a healthy bus, and a wrong opcode is the only
 * explanation for that which does not require the chip to be broken. So the opcode becomes a
 * parameter, and protocol_scan() walks a table of them.
 */
static bool
fts_read_raw(uint8_t op, uint16_t address, uint8_t *data, uint32_t length)
{
	uint8_t buf[FTS_FIFO_DEPTH + 1];
	bool ok = false;

	if (length > sizeof(buf) - 1) {
		return false;
	}
	i2c_start();
	if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(op) &&
	    i2c_write_byte((uint8_t)(address >> 8)) &&
	    i2c_write_byte((uint8_t)(address & 0xff))) {
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
			for (uint32_t i = 0; i <= length; i++) {
				buf[i] = i2c_read_byte(i < length);
			}
			memcpy(data, buf + 1, length);
			ok = true;
		}
	}
	i2c_stop();
	return ok;
}

/* ftsIO.c:writeCmdU16 -- the command, both address bytes and the data in one transaction. */
static bool
fts_write_registers(uint16_t address, const uint8_t *data, uint32_t length)
{
	bool ok;

	i2c_start();
	ok = i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(FTS_CMD_HW_REG) &&
	    i2c_write_byte((uint8_t)(address >> 8)) &&
	    i2c_write_byte((uint8_t)(address & 0xff));
	for (uint32_t i = 0; ok && i < length; i++) {
		ok = i2c_write_byte(data[i]);
	}
	i2c_stop();
	return ok;
}

/*
 * A command is a single byte out and nothing back -- ftsIO.c:fts_writeCmd with a one-byte buffer.
 * The event read is the same shape with bytes coming back, and the sense-on command is a bare
 * command with no address and no answer.
 */
static bool
fts_command(uint8_t command)
{
	bool ok;

	i2c_start();
	ok = i2c_write_byte((uint8_t)(bus_address << 1)) && i2c_write_byte(command);
	i2c_stop();
	return ok;
}

/*
 * One event out of the FIFO: a single command byte, 0x85, then eight bytes back (fts.c:
 * fts_event_handler). There is no dummy byte here -- that is only for hardware registers -- and no
 * address. The first byte of the event is its id, and 0x00 means the FIFO is empty.
 */
static bool
fts_read_event(uint8_t *event)
{
	bool ok = false;

	i2c_start();
	if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(FTS_CMD_READ_ONE)) {
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
			for (uint32_t i = 0; i < FTS_EVENT_SIZE; i++) {
				event[i] = i2c_read_byte(i + 1 < FTS_EVENT_SIZE);
			}
			ok = true;
		}
	}
	i2c_stop();
	return ok;
}

/* ---- the chip ----------------------------------------------------------------------------------- */

static uint32_t flip;              /* boot-arg umi_touch_flip */

static struct {
	uint32_t polls, reports, touches, bad, no_ack, last_x, last_y;
} seen;

static struct {
	bool down;
	uint32_t x, y;
} contact[FTS_MAX_TOUCHES];

#define PANEL_X 1080               /* fts,x-max */
#define PANEL_Y 2340               /* fts,y-max */

static void
report_touch(uint32_t id, bool down, uint32_t x, uint32_t y)
{
	static unsigned shown;

	if (id >= FTS_MAX_TOUCHES) {
		return;
	}
	if (!down) {
		if (contact[id].down) {
			contact[id].down = false;
			redwood_touch_post(REDWOOD_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
		return;
	}
	seen.touches++;
	seen.last_x = x;
	seen.last_y = y;
	switch (flip) {
	case 1: x = PANEL_X - 1 - x; break;
	case 2: y = PANEL_Y - 1 - y; break;
	case 3: x = PANEL_X - 1 - x; y = PANEL_Y - 1 - y; break;
	}
	x = (uint32_t)(((uint64_t)x * 65535) / (PANEL_X - 1));
	y = (uint32_t)(((uint64_t)y * 65535) / (PANEL_Y - 1));
	if (!contact[id].down) {
		contact[id].down = true;
		redwood_touch_post(REDWOOD_TOUCH_DOWN, id, x, y);
		if (shown++ < 8) {
			printf("umi_touch: finger %u down at %u, %u\n", id, x, y);
		}
	} else if (contact[id].x != x || contact[id].y != y) {
		redwood_touch_post(REDWOOD_TOUCH_MOVE, id, x, y);
	}
	contact[id].x = x;
	contact[id].y = y;
}

/*
 * One event out of the FIFO, in the layout fts.c reads it (fts_enter_pointer_event_handler and
 * fts_leave_pointer_event_handler):
 *   event[0]   event id: 0x03 enter, 0x04 leave, 0x05 motion (the same handler as enter)
 *   event[1]   [7:4] how many fingers are down, [3:0] this finger's slot
 *   event[2]   x bits 11..4
 *   event[3]   y bits 11..4
 *   event[4]   [7:4] x bits 3..0, [3:0] y bits 3..0
 *   event[5]   [4:0] touch major, [7:6] touch size
 *   event[6]   touch minor
 *
 * Unlike the FocalTech report this file used to parse, the chip sends one event per finger per
 * change and nothing at all when nothing happens -- so a lifted finger is an 0x04 event with its
 * slot in event[1], not a slot that has gone missing from a snapshot.
 */
static void
parse_event(const uint8_t *event)
{
	uint32_t id = event[1] & 0x0f;
	uint32_t x = ((uint32_t)event[2] << 4) | ((event[4] & 0xf0) >> 4);
	uint32_t y = ((uint32_t)event[3] << 4) | (event[4] & 0x0f);

	seen.reports++;
	report_touch(id, event[0] != EVENTID_LEAVE_POINTER, x, y);
}

/* ---- bring-up ----------------------------------------------------------------------------------- */

static void
pins_for_i2c(void)
{
	line_release(pin_sda);
	scl_high();
	gpio_output(PIN_RESET, false, PULL_UP, 8);      /* held in reset until the chip is asked for */
	gpio_configure(PIN_INT, 0, PULL_UP, 8, false);
}

/*
 * Can this pad drive the line both ways, and does the line come back when it is let go? A read of
 * 0x00 with an acknowledge on every byte is not proof that a chip answered: if SDA is held low by
 * anything at all -- an unpowered chip, a chip still in reset, a short -- then the acknowledge bit
 * reads low too and every read returns zero. Driving the line low, then high, then letting the
 * pull-up have it, and printing what the input reads each time, separates "the pad works and
 * something else holds the line" from "this is not the line we think it is".
 */
static void
line_test(void)
{
	unsigned pull;

	/* First: can the pad drive the line both ways at all? */
	for (int i = 0; i < 2; i++) {
		unsigned pin = i ? PIN_SCL : PIN_SDA;

		gpio_output(pin, false, PULL_NONE, 2);
		printf("umi_touch: ... gpio%u driven low -> %u\n", pin, gpio_get(pin));
		gpio_output(pin, true, PULL_NONE, 2);
		printf("umi_touch: ... gpio%u driven high -> %u\n", pin, gpio_get(pin));
	}
	/*
	 * Then: which of the four values of the pull field holds a released line up? The usual
	 * description is 0 none, 1 pull-down, 2 keeper, 3 pull-up, and the field does read back what
	 * is written -- but a line that decays to 0 within 200 us of the driver being switched off
	 * is a line with nothing holding it up, whatever the register says. Trying all four on the
	 * device answers it without relying on that description being right for this SoC.
	 */
	for (pull = 0; pull <= 3; pull++) {
		gpio_configure(PIN_SDA, 0, pull, 2, false);
		gpio_configure(PIN_SCL, 0, pull, 2, false);
		sleep_us(200);
		printf("umi_touch: ... pull=%u cfg %#x/%#x, released: sda %u scl %u\n", pull,
		    rd(GPIO_CFG(PIN_SDA)), rd(GPIO_CFG(PIN_SCL)), gpio_get(PIN_SDA), gpio_get(PIN_SCL));
	}
	/* leave the pair as the driver will use them */
	gpio_configure(PIN_SDA, 0, PULL_UP, 2, false);
	gpio_configure(PIN_SCL, 0, PULL_UP, 2, false);
}

/*
 * Who is this, really, and is it running?
 *
 * Both of the vendor driver's compile-time variants are asked, because nothing here says which one
 * umi is: the digital chip ID and the firmware version live at 0x0004/0x0008 without FTM3_CHIP and
 * at 0x0007/0x000a with it, and the expected ID pairs are 36 70 and 39 6c. Whichever answers is
 * this phone's part -- and the answer is also the first proof that the protocol is right at all:
 * a 16-bit address, command 0xB6, and one dummy byte in front of the data.
 */
static bool
read_chip_id(uint8_t *high, uint8_t *low)
{
	static const struct {
		uint16_t id, fw;
		uint8_t id0, id1;
		const char *name;
	} variant[2] = {
		{ FTS_CHIP_ID_ADDR_A, FTS_FW_VER_ADDR_A, FTS_ID_0_A, FTS_ID_1_A, "without FTM3_CHIP" },
		{ FTS_CHIP_ID_ADDR_B, FTS_FW_VER_ADDR_B, FTS_ID_0_B, FTS_ID_1_B, "with FTM3_CHIP" },
	};
	bool any = false;

	for (unsigned v = 0; v < 2; v++) {
		uint8_t id[2] = { 0, 0 };
		uint8_t fw[FTS_FW_VER_BYTES] = { 0, 0 };
		bool id_ok, fw_ok;

		id_ok = fts_read_registers(variant[v].id, id, sizeof(id));
		fw_ok = fts_read_registers(variant[v].fw, fw, sizeof(fw));
		if (!id_ok) {
			continue;
		}
		any = true;
		printf("umi_touch: id at %#06x %s: %02x %02x%s; fw at %#06x: %02x %02x%s\n",
		    variant[v].id, variant[v].name, id[0], id[1],
		    (id[0] == variant[v].id0 && id[1] == variant[v].id1) ? " -- that is this part" : "",
		    variant[v].fw, fw[0], fw_ok ? fw[1] : 0, fw_ok ? "" : " (read failed)");
		if (id[0] == variant[v].id0 && id[1] == variant[v].id1) {
			*high = id[0];
			*low = id[1];
			return true;
		}
	}
	return any && false;
}

/* One try at one pin order: reset the chip, let its firmware come up, ask who it is. */
static bool
probe(unsigned sda, unsigned scl, bool verbose)
{
	uint8_t high = 0, low = 0;

	pin_sda = sda;
	pin_scl = scl;
	if (verbose) {
		trace("one try, sda and scl as given");
	}
	pins_for_i2c();
	if (verbose) {
		trace("the two lines open drain, reset held low");
	}
	i2c_bus_recover();
	if (verbose) {
		trace("the bus looked at");
	}

	/*
	 * Which level means "in reset" is not written down. The overlay gives the line as
	 * <&tlmm 38 0>, which reads as active high, and an active-high reset is asserted by driving
	 * the line high -- yet the vendor's own fts_reset_proc ends with the line high, which is
	 * what an active-low reset looks like when the chip is let go. One of those leaves the chip
	 * held in reset, and a chip in reset does not acknowledge anything, which is exactly what
	 * the bus reports. So umi_touch_reset chooses the level the sequence settles at: 1 high (as
	 * this driver has always done), 0 low, 2 leaves the line alone. Either way the pulse goes
	 * the other way first, so there is a real edge.
	 */
	if (reset_mode != 2) {
		bool settle_high = reset_mode != 0;

		gpio_set(PIN_RESET, !settle_high);
		sleep_ms(2);
		if (verbose) {
			trace("2 ms pulsed the other way");
		}
		gpio_set(PIN_RESET, settle_high);
		if (verbose) {
			trace(settle_high ? "reset settled high" : "reset settled low");
		}
	} else if (verbose) {
		trace("reset left where it was");
	}
	sleep_ms(settle_ms);              /* the vendor's fts_reset_proc, then time to boot the app */
	if (verbose) {
		trace("200 ms later");
	}
	/*
	 * The vendor driver's fts_chip_powercycle2, in its order: assert the reset (low), bring the
	 * supplies up, give the regulators 20 ms to reach their value, wait another 20 ms, then
	 * release the reset (high). So the reset line is active low and the chip is out of reset at
	 * the level this driver has always settled at -- which is also what the bus said when it
	 * was measured: reset low reads 0xff and writes are not acknowledged, reset high reads
	 * 0x00 and they are. Nothing needs to be started by hand; there is no 0x55/0xAA handshake
	 * on this part, that was FocalTech's and this is not a FocalTech.
	 */
	if (reset_mode != 2) {
		bool settle_high = reset_mode != 0;

		gpio_set(PIN_RESET, !settle_high);
		sleep_ms(2);
		if (verbose) {
			trace("2 ms pulsed the other way");
		}
		gpio_set(PIN_RESET, settle_high);
		if (verbose) {
			trace(settle_high ? "reset settled high" : "reset settled low");
		}
	} else if (verbose) {
		trace("reset left where it was");
	}
	sleep_ms(settle_ms);              /* the chip's own boot, then its firmware */
	if (verbose) {
		trace("settle time later");
	}

	for (int attempt = 0; attempt < 5; attempt++) {
		if (verbose) {
			trace("asking who it is");
		}
		if (read_chip_id(&high, &low)) {
			if (verbose) {
				printf("umi_touch: chip at %#x on sda gpio%u / scl gpio%u: id %02x %02x (after %d attempt(s))\n",
				    I2C_ADDRESS, pin_sda, pin_scl, high, low, attempt + 1);
			}
			return true;
		}
		seen.no_ack++;
		if (verbose) {
			printf("umi_touch: ... attempt %d: nothing at the id registers\n", attempt + 1);
		}
		sleep_ms(50);
	}
	if (verbose) {
		printf("umi_touch: no answer at %#x on sda gpio%u / scl gpio%u\n",
		    I2C_ADDRESS, pin_sda, pin_scl);
	}
	return false;
}

static void
register_dump(void)
{
	uint8_t line[16];

	printf("umi_touch: hardware registers 0x0000..0x00ff at %#x (sda gpio%u, scl gpio%u):\n",
	    I2C_ADDRESS, pin_sda, pin_scl);
	for (unsigned base = 0; base < 256; base += 16) {
		for (unsigned i = 0; i < 16; i++) {
			line[i] = 0xff;
			if (!fts_read_registers((uint16_t)(base + i), &line[i], 1)) {
				line[i] = 0xfe;    /* no acknowledge */
			}
		}
		printf("umi_touch: %04x: %02x %02x %02x %02x %02x %02x %02x %02x "
		    "%02x %02x %02x %02x %02x %02x %02x %02x\n", base,
		    line[0], line[1], line[2], line[3], line[4], line[5], line[6], line[7],
		    line[8], line[9], line[10], line[11], line[12], line[13], line[14], line[15]);
	}
}

/*
 * Which registers of this chip are alive? Reading the whole map twice and printing only what
 * changed answers that without knowing what any of it means: a chip whose core is running has
 * counters and status bits that move on their own, and one that answers only from a bus-powered
 * slave interface is frozen. The map this driver gets is four non-zero bytes out of 256, so the
 * question worth asking is not what they are but whether anything moves at all.
 */
static void
register_diff(void)
{
	static uint8_t before[256], after[256];

	for (unsigned i = 0; i < 256; i++) {
		before[i] = 0xff;
		if (!fts_read_registers((uint16_t)i, &before[i], 1)) {
			before[i] = 0xfe;
		}
	}
	for (int round = 1; round <= 3; round++) {
		unsigned changed = 0;

		sleep_ms(200);
		for (unsigned i = 0; i < 256; i++) {
			after[i] = 0xff;
			if (!fts_read_registers((uint16_t)i, &after[i], 1)) {
				after[i] = 0xfe;
			}
		}
		for (unsigned i = 0; i < 256; i++) {
			if (before[i] != after[i]) {
				printf("umi_touch:   %04x: %02x -> %02x\n", i, before[i], after[i]);
				changed++;
				before[i] = after[i];
			}
		}
		printf("umi_touch: round %d: %u register(s) moved in 200 ms\n", round, changed);
	}
}

/*
 * Is the answer stable, and does it depend on the dummy byte?
 *
 * This used to write a byte and read it back to see whether anything behind the I2C slave was
 * alive. That question is answered: the protocol was wrong, not the chip, and nothing here writes
 * to a register whose meaning is not known -- 0x0024 is the interface control register and writing
 * the wrong value to it switches the part to SPI. So the test reads the ID and the firmware version
 * five times instead, which is what proves the dummy byte is being dropped in the right place: a
 * misaligned read shifts by one and the same two bytes come back as a different pair every time.
 */
static void
write_test(void)
{
	for (int i = 0; i < 5; i++) {
		uint8_t id[2] = { 0, 0 }, fw[2] = { 0, 0 };

		(void)fts_read_registers(FTS_CHIP_ID_ADDR_A, id, 2);
		(void)fts_read_registers(FTS_FW_VER_ADDR_A, fw, 2);
		printf("umi_touch: read %d: id %02x %02x fw %02x %02x\n", i, id[0], id[1], fw[0], fw[1]);
		sleep_ms(20);
	}
}

/*
 * A register read that keeps the bytes it would normally throw away, and says how many it dropped.
 *
 * fts_read_registers always drops one leading byte, because DUMMY_HW_REG is 1. If that is wrong for
 * this part -- 0, or 2 -- every value read is shifted and the answer looks like a dead chip rather
 * than a misread one. Printing all of them with the count varying is the only way to tell a wrong
 * dummy count from a chip that really does answer 00.
 */
static bool
fts_read_verbose(uint16_t address, unsigned length, unsigned dummy, const char *what)
{
	uint8_t buf[FTS_FIFO_DEPTH + 2];
	unsigned total = length + dummy;
	bool ok = false;

	if (total > sizeof(buf)) {
		return false;
	}
	i2c_start();
	if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(FTS_CMD_HW_REG) &&
	    i2c_write_byte((uint8_t)(address >> 8)) &&
	    i2c_write_byte((uint8_t)(address & 0xff))) {
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
			for (unsigned i = 0; i < total; i++) {
				buf[i] = i2c_read_byte(i + 1 < total);
			}
			ok = true;
		}
	}
	i2c_stop();
	if (!ok) {
		printf("umi_touch: %s at %#06x: not acknowledged\n", what, address);
		return false;
	}
	printf("umi_touch: %s at %#06x, drop %u of %u:", what, address, dummy, total);
	for (unsigned i = 0; i < total; i++) {
		printf(" %02x", buf[i]);
	}
	printf("  (sda %u scl %u)\n", gpio_get(pin_sda), gpio_get(pin_scl));
	return true;
}

/*
 * The eight bits of one byte, sampled twice: while the clock is high, which is when the value
 * counts, and while it is low, which is when the slave is allowed to change it.
 *
 * If every high-phase sample is 0 and every low-phase sample is 1, the chip is genuinely sending
 * 00 00 and the read path works. If both are 0, something is holding SDA down and no read on this
 * bus can return anything but zeros -- which is the failure this driver already had once, and it
 * looks identical from the outside.
 */
static void
sample_bits(uint16_t address)
{
	unsigned high_phase = 0, low_phase = 0;
	bool ok = false;

	i2c_start();
	if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
	    i2c_write_byte(FTS_CMD_HW_REG) &&
	    i2c_write_byte((uint8_t)(address >> 8)) &&
	    i2c_write_byte((uint8_t)(address & 0xff))) {
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
			ok = true;
		}
	}
	printf("umi_touch: bit samples at %#06x, %s: high ", address, ok ? "acked" : "not acked");
	for (int bit = 7; ok && bit >= 0; bit--) {
		i2c_delay();
		scl_high();
		i2c_delay();
		if (gpio_get(pin_sda)) {
			high_phase++;
		}
		printf("%u", gpio_get(pin_sda));
		line_low(pin_scl);
		i2c_delay();
		if (gpio_get(pin_sda)) {
			low_phase++;
		}
	}
	printf("  low ");
	if (ok) {
		/* second pass: sample the same byte again, this time only while the clock is low */
		i2c_stop();
		i2c_start();
		if (i2c_write_byte((uint8_t)(bus_address << 1)) &&
		    i2c_write_byte(FTS_CMD_HW_REG) &&
		    i2c_write_byte((uint8_t)(address >> 8)) &&
		    i2c_write_byte((uint8_t)(address & 0xff))) {
			i2c_start();
			if (i2c_write_byte((uint8_t)(bus_address << 1 | 1))) {
				for (int bit = 7; bit >= 0; bit--) {
					printf("%u", gpio_get(pin_sda));
					i2c_delay();
					scl_high();
					i2c_delay();
					line_low(pin_scl);
					i2c_delay();
				}
			}
		}
	}
	i2c_stop();
	printf("  (%u high, %u low out of 8)\n", high_phase, low_phase);
}

static bool panel_supply_on(const char *name);   /* pm8150_s4 / smpa4, defined with the RPMh code below */
static bool panel_supply_off(const char *name);

/*
 * The chip answers its address and returns zero for everything. Two explanations survive; this
 * function tells them apart.
 *
 *   (a) The opcode is wrong. A slave acknowledges any byte sent to its address, so 0xB6 being the
 *       wrong "read a hardware register" command for this part looks exactly like a dead part:
 *       no error anywhere, zeros for data. Xiaomi's part is an ST FingerTipS (st_fts_j2.ftb), but
 *       the header the constants came from is one vendor's build of that driver, not this phone's.
 *   (b) The core is not running. The I/O ring is powered from the 1.8 V the pair's pull-ups hang
 *       from -- ldoc1, proved by run 152817, where asking RPMh for that rail alone took sda and
 *       scl from 0 to 1 with no pin was touched -- and a part in that state acknowledges its
 *       address while every register reads zero.
 *
 * A first answer to (a) is cheap: the same register read through sixteen opcodes. A first answer
 * to (b) is cheaper still: the whole 256-byte register file, because a core that is running has
 * *something* non-zero in it somewhere. Then a write and a read-back at sixteen addresses, because
 * a register file that reads zero everywhere and also refuses writes is not a register file.
 */
static void
protocol_scan(void)
{
	/*
	 * 0xb6 is the vendor header's value. The rest are its neighbours, the framebuffer opcode, the
	 * FIFO opcodes, and the two "page select" style bytes other ST parts use. Anything that comes
	 * back non-zero is the answer; if all sixteen return zero, the opcode is not the problem.
	 */
	static const uint8_t opcodes[] = {
		0xb6, 0xb7, 0xb5, 0xb4, 0xd0, 0xd1, 0xc0, 0xc1,
		0x80, 0x86, 0x90, 0xa0, 0xb0, 0xe0, 0x5a, 0x4b,
	};
	uint8_t data[8];

	printf("umi_touch: --- is the acknowledge ours? 0x48 must stay silent ---\n");
	printf("umi_touch:   0x48: %s\n", i2c_address_present(0x48) ? "ACKNOWLEDGES" : "silent");
	printf("umi_touch:   0x49: %s\n", i2c_address_present(0x49) ? "acknowledges" : "SILENT");

	printf("umi_touch: --- the id register through sixteen opcodes ---\n");
	for (unsigned i = 0; i < sizeof(opcodes); i++) {
		memset(data, 0, sizeof(data));
		if (!fts_read_raw(opcodes[i], FTS_CHIP_ID_ADDR_A, data, 4)) {
			printf("umi_touch: op %02x: the address or a command byte was not acknowledged\n",
			    opcodes[i]);
			continue;
		}
		printf("umi_touch: op %02x: %02x %02x %02x %02x   sda %u scl %u\n", opcodes[i],
		    data[0], data[1], data[2], data[3], gpio_get(pin_sda), gpio_get(pin_scl));
	}

	printf("umi_touch: --- the whole register file, 0x00..0xff, eight bytes a group ---\n");
	for (unsigned row = 0; row < 0x100; row += 32) {
		printf("umi_touch: %04x:", row);
		for (unsigned i = 0; i < 32; i += 8) {
			memset(data, 0, sizeof(data));
			(void)fts_read_raw(FTS_CMD_HW_REG, (uint16_t)(row + i), data, 8);
			printf(" %02x%02x%02x%02x%02x%02x%02x%02x",
			    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		}
		printf("\n");
	}

	/*
	 * A register that reads zero and also refuses a write is not a register. Sixteen addresses
	 * across the map, each written with 0x5a and read straight back -- the read-back is what makes
	 * it a test: if even one address holds the value, the register file is there and the problem
	 * is which addresses this chip uses.
	 */
	printf("umi_touch: --- write 0x5a and read it back, sixteen addresses ---\n");
	for (unsigned address = 0x0000; address < 0x0100; address += 0x10) {
		uint8_t want = 0x5a;
		uint8_t back[2] = { 0, 0 };

		if (!fts_write_registers((uint16_t)address, &want, 1)) {
			printf("umi_touch: %04x: the write was not acknowledged\n", address);
			continue;
		}
		if (!fts_read_registers((uint16_t)address, back, 1)) {
			printf("umi_touch: %04x: written, the read was not acknowledged\n", address);
			continue;
		}
		printf("umi_touch: %04x <- 5a, read back %02x %s\n", address, back[0],
		    back[0] == want ? "<- it stuck" : "");
	}

	/*
	 * The interrupt line has been low in every run: by the vendor driver's reading that means the
	 * chip has an event queued. Sense-on is the command that starts the panel scan; if the core is
	 * running at all, either the FIFO answers or the interrupt line moves.
	 */
	printf("umi_touch: --- sense on, then the fifo ---\n");
	printf("umi_touch: command %02x: %s\n", FTS_CMD_SENSE_ON,
	    fts_command(FTS_CMD_SENSE_ON) ? "acknowledged" : "not acknowledged");
	sleep_ms(50);
	printf("umi_touch: after sense on: int %u\n", gpio_get(PIN_INT));
	for (unsigned i = 0; i < 4; i++) {
		uint8_t event[FTS_EVENT_SIZE];

		if (!fts_read_event(event)) {
			printf("umi_touch: fifo read not acknowledged\n");
			break;
		}
		printf("umi_touch: fifo %u: %02x %02x %02x %02x %02x %02x %02x %02x\n", i,
		    event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
		if (event[0] == EVENTID_NO_EVENT) {
			break;
		}
	}

	/*
	 * The last state that would look like this from outside is sleep. Android was running before
	 * this kernel, its driver had the part suspended, and a part in its low-power state can keep
	 * the i2c block answering while the register file reads zero -- which is why sense-on is sent
	 * before this read and not after it. Three reads in a row, because a register that is really
	 * there answers the same thing every time and a shift or a timing problem does not.
	 */
	printf("umi_touch: --- the id again, three times, after sense on ---\n");
	for (unsigned i = 0; i < 3; i++) {
		memset(data, 0, sizeof(data));
		if (!fts_read_registers(FTS_CHIP_ID_ADDR_A, data, 2)) {
			printf("umi_touch: id %u: not acknowledged\n", i);
			continue;
		}
		printf("umi_touch: id %u: %02x %02x   sda %u scl %u int %u\n", i, data[0], data[1],
		    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_INT));
	}
	printf("umi_touch: at the end: sda %u scl %u int %u\n",
	    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_INT));
}

/*
 * Everything the read path could still have wrong, in one boot.
 *
 * The command byte, the 16-bit address and one dummy byte are what the vendor driver does, and with
 * them this part answers 00 00 at both ID addresses and 00 00 at the firmware version -- which the
 * vendor driver itself reads as "no firmware running in the chip". Before believing that, this
 * settles whether the reads are real: the dummy count is varied, the raw bytes are printed, the
 * bits are sampled at both clock phases, the event FIFO is read (the interrupt line is low, so the
 * chip believes it has something to say), and a system reset is sent to see whether a controller
 * ready event turns up afterwards.
 */
static void
sweep(void)
{
	uint8_t event[FTS_EVENT_SIZE];
	uint8_t value;

	/*
	 * Before anything is measured: clear whatever the previous run left on this pair. A run that
	 * ended in the middle of a byte leaves the chip stretching the clock, and every reading after
	 * that is zeros for a reason that has nothing to do with the chip.
	 */
	i2c_bus_recover();
	printf("umi_touch: the pair before anything: sda %u scl %u\n",
	    gpio_get(pin_sda), gpio_get(pin_scl));

	printf("umi_touch: --- the read path, with the dummy count varied ---\n");
	for (unsigned dummy = 0; dummy <= 2; dummy++) {
		(void)fts_read_verbose(FTS_CHIP_ID_ADDR_A, 2, dummy, "id");
		(void)fts_read_verbose(FTS_CHIP_ID_ADDR_B, 2, dummy, "id");
	}
	(void)fts_read_verbose(FTS_FW_VER_ADDR_A, 2, 1, "fw");
	(void)fts_read_verbose(0x0000, 8, 1, "map");

	printf("umi_touch: --- are the zero bits driven, or held down? ---\n");
	sample_bits(FTS_CHIP_ID_ADDR_A);

	/*
	 * The one thing none of the above tests: whether the chip is powered at all.
	 *
	 * The overlay's fts@49 has avdd-supply = <&pm8150_s4> and fts,pwr-reg-name = "avdd", and
	 * vdd-supply is PHANDLE_NONE -- so the rail this part runs from is the main PMIC's SMPS4,
	 * which RPMh knows as "smpa4", and there is no second rail to worry about. Nothing in this
	 * port had ever asked RPMh for it. What it had asked for is ldoc1, on the strength of
	 * fts,bus-reg-name = "vdd"; that name points at a supply that does not exist.
	 *
	 * A part whose I/O ring is fed from an always-on 1.8 V and whose core is not answers its
	 * address, holds every data bit at zero and looks exactly like a part with no firmware. So
	 * this raises the rail, gives the chip a power-on reset it has never had, and reads the id
	 * again -- the before and after are two lines apart in the same log.
	 */
	printf("umi_touch: --- avdd: pm8150_s4 (smpa4), then the id again ---\n");
	{
		static const char *const rails[] = { "smpa4", "smpa5" };
		bool answered = false;

		for (unsigned r = 0; r < 2 && !answered; r++) {
			uint8_t id[2] = { 0, 0 };

			if (!panel_supply_on(rails[r])) {
				continue;
			}
			sleep_ms(5);
			gpio_output(PIN_RESET, false, PULL_NONE, 2);    /* in reset while the rail settles */
			sleep_ms(20);
			gpio_output(PIN_RESET, true, PULL_NONE, 2);
			sleep_ms(settle_ms);
			printf("umi_touch: after %s and a reset pulse: sda %u scl %u reset %u int %u\n",
			    rails[r], gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_RESET),
			    gpio_get(PIN_INT));
			(void)fts_read_verbose(FTS_CHIP_ID_ADDR_A, 2, 1, "id after avdd");
			(void)fts_read_verbose(FTS_CHIP_ID_ADDR_B, 2, 1, "id after avdd");
			(void)fts_read_verbose(FTS_CHIP_ID_ADDR_A, 2, 0, "id after avdd, no dummy");
			/*
			 * The same two bytes through the normal path, so that "is the chip there" is a
			 * value this code can decide on rather than a line a person has to read.
			 */
			if (fts_read_registers(FTS_CHIP_ID_ADDR_A, id, 2)) {
				answered = id[0] != 0 || id[1] != 0;
				printf("umi_touch: %s: id %02x %02x -> %s\n", rails[r], id[0], id[1],
				    answered ? "a chip that answers" : "still all zero");
			}
			if (answered) {
				sample_bits(FTS_CHIP_ID_ADDR_A);
			}
		}
		if (!answered) {
			printf("umi_touch: avdd did not change the answer; the readings above are from a part "
			    "that either has no firmware or is not the thing at %#x\n", I2C_ADDRESS);
		}
	}

	/*
	 * A read of 0x00 is ambiguous in one more way: it can mean "the register file says zero" or
	 * "the register file is not there". A write that comes back is the difference. IER is the one
	 * register the vendor driver both writes and relies on, and it exists at 0x001c on one chip
	 * variant and 0x002c on the other, so both are tried with the same value.
	 */
	printf("umi_touch: --- does a write stick? ---\n");
	{
		static const uint16_t regs[] = { 0x001c, 0x002c };
		uint8_t back[2];

		for (unsigned r = 0; r < 2; r++) {
			uint8_t want = FTS_IER_ENABLE;

			if (!fts_write_registers(regs[r], &want, 1)) {
				printf("umi_touch: write %#06x <- %02x: not acked\n", regs[r], want);
				continue;
			}
			back[0] = back[1] = 0;
			if (!fts_read_registers(regs[r], back, 2)) {
				printf("umi_touch: write %#06x <- %02x: acked, read back not acked\n",
				    regs[r], want);
				continue;
			}
			printf("umi_touch: write %#06x <- %02x: read back %02x %02x -> %s\n", regs[r], want,
			    back[0], back[1],
			    back[0] == want ? "the register file is alive"
			    : "the write did not stick");
		}
	}

	printf("umi_touch: --- the event fifo, before anything ---\n");
	for (unsigned i = 0; i < 4; i++) {
		if (!fts_read_event(event)) {
			printf("umi_touch: fifo read not acknowledged\n");
			break;
		}
		printf("umi_touch: fifo %u: %02x %02x %02x %02x %02x %02x %02x %02x\n", i,
		    event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
		if (event[0] == EVENTID_NO_EVENT) {
			break;
		}
	}

	printf("umi_touch: --- a system reset over i2c, then the fifo again ---\n");
	value = FTS_SYS_RESET_VAL_A;
	printf("umi_touch: reset %#06x <- %02x: %s\n", FTS_SYS_RESET_ADDR_A, value,
	    fts_write_registers(FTS_SYS_RESET_ADDR_A, &value, 1) ? "acked" : "not acked");
	sleep_ms(100);
	for (unsigned i = 0; i < 4; i++) {
		if (!fts_read_event(event)) {
			printf("umi_touch: fifo read not acknowledged\n");
			break;
		}
		printf("umi_touch: after reset fifo %u: %02x %02x %02x %02x\n", i,
		    event[0], event[1], event[2], event[3]);
		if (event[0] == EVENTID_NO_EVENT) {
			break;
		}
	}
	value = FTS_SYS_RESET_VAL_B;
	printf("umi_touch: reset %#06x <- %02x: %s\n", FTS_SYS_RESET_ADDR_B, value,
	    fts_write_registers(FTS_SYS_RESET_ADDR_B, &value, 1) ? "acked" : "not acked");
	sleep_ms(100);
	for (unsigned i = 0; i < 4; i++) {
		if (!fts_read_event(event)) {
			printf("umi_touch: fifo read not acknowledged\n");
			break;
		}
		printf("umi_touch: after the other reset fifo %u: %02x %02x %02x %02x\n", i,
		    event[0], event[1], event[2], event[3]);
		if (event[0] == EVENTID_NO_EVENT) {
			break;
		}
	}
	printf("umi_touch: at the end: sda %u scl %u int %u\n",
	    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_INT));
}

static void
poll_report(void)
{
	static uint8_t event[FTS_EVENT_SIZE];
	static unsigned shown_raw;

	seen.polls++;
	/*
	 * Drain the FIFO: the chip queues one event per finger per change, up to FIFO_DEPTH of them,
	 * and 0x00 means empty. Reading exactly one event per look and stopping there was what the
	 * FocalTech snapshot report wanted; here it would leave a queue behind and the touch would
	 * lag further behind the finger with every look.
	 */
	for (unsigned i = 0; i < FTS_FIFO_DEPTH; i++) {
		if (!fts_read_event(event)) {
			seen.bad++;
			return;
		}
		if (event[0] == EVENTID_NO_EVENT) {
			return;
		}
		if (shown_raw < 8) {
			shown_raw++;
			printf("umi_touch: event %02x %02x %02x %02x %02x %02x %02x %02x\n",
			    event[0], event[1], event[2], event[3],
			    event[4], event[5], event[6], event[7]);
		}
		switch (event[0]) {
		case EVENTID_ENTER_POINTER:
		case EVENTID_MOTION_POINTER:
		case EVENTID_LEAVE_POINTER:
			parse_event(event);
			break;
		default:
			break;          /* status, errors and the rest are not needed to move a cursor */
		}
	}
}

/* ---- what RPMh can switch (dt: reserved-memory cmd_db@80860000) ---------------------------------- */

/*
 * The touch controller is not answering, and with a real high level on both lines rather than a
 * pull-up that is not there, that means no acknowledge rather than a misread one. The panel's
 * overlay gives fts@49 avdd-supply = pm8150_s4 in one variant and pm8150_s5 in another -- PMIC
 * SMPS rails, not the gpio-gated disp_vddio switch this driver already reads high. Nothing in
 * this port votes a PMIC regulator yet, so the rail is off.
 *
 * The boot loader leaves a list of everything RPMh can switch, with the address each one is
 * switched at (RPMh's own cmd-db, which redwood_adreno.c reads the same way for gfx.lvl and
 * cx.lvl). Printing it is read-only and says which of those rails exist and where, before any
 * run decides to raise one.
 */
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);

#define CMD_DB_PHYS              0x80860000UL
#define CMD_DB_SIZE              0x20000UL

struct cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

static void
dump_cmd_db(void)
{
	const struct cmd_db_header *db;

	db = (const struct cmd_db_header *)ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);
	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		printf("umi_touch: cmd_db at %#x has no magic (%02x %02x %02x %02x)\n",
		    (uint32_t)CMD_DB_PHYS, db->magic[0], db->magic[1], db->magic[2], db->magic[3]);
		return;
	}
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry;

		if (slave->id == 0 || slave->count == 0 || slave->count > 512) {
			continue;
		}
		printf("umi_touch: cmd_db slave %u (id %u): %u entries\n", s, slave->id, slave->count);
		entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);
		for (unsigned i = 0; i < slave->count; i++) {
			char id[9];

			memcpy(id, entry[i].id, 8);
			id[8] = 0;
			printf("umi_touch:   %-8s %#010x  %u bytes\n", id, entry[i].address, entry[i].length);
		}
	}
}

/*
 * ---- asking RPMh for the bus supply ---------------------------------------------------------------
 *
 * The same code as bsd/dev/redwood_goodix.c's io_supply_on(), which does this on the POCO X5 Pro for
 * its own touch controller, and it is here rather than shared because the two drivers' register maps
 * otherwise have nothing in common and a header would be one function in a file of its own.
 *
 * The rail is pm8150a_l1: the panel overlay names it as fts@49's vdd-supply, and fts,bus-reg-name is
 * "vdd" -- the bus supply, the one the pair's pull-ups hang from. Without it there is no high level
 * on SDA or SCL at all, which is what this driver measured before it could talk to anything. Its
 * device-tree node is rpmh-regulator-ldoc1/regulator-pm8150a-l1, so the name RPMh knows it by is
 * "ldoc1" -- 0x42e00 in the command database -- at 1 800 000 uV, fixed, and nothing else in the tree
 * refers to it.
 */
#define RSC_PHYS                 0x18220000UL   /* the application processor's "DRV 2" */
#define RSC_SIZE                 0x10000UL
#define TCS(reg)                 (rsc + 0xd00 + (reg))      /* command slot set 0, an "active" one */
#define TCS_IRQ_ENABLE           0x00
#define TCS_IRQ_STATUS           0x04
#define TCS_IRQ_CLEAR            0x08
#define TCS_WAIT_FOR_COMPLETION  0x10
#define TCS_CONTROL              0x14
#define TCS_CONTROL_ENABLE       (1u << 16)
#define TCS_CONTROL_TRIGGER      (1u << 24)
#define TCS_STATUS               0x18            /* 0 while busy */
#define TCS_CMD_ENABLE           0x1c
#define TCS_CMD_MSGID(n)         (0x30 + 20 * (n))
#define TCS_CMD_ADDR(n)          (0x34 + 20 * (n))
#define TCS_CMD_DATA(n)          (0x38 + 20 * (n))
#define MSGID_WRITE_4_BYTES      0x10008
#define MSGID_RESPONSE           0x100

#define VRM_VOLTAGE              0               /* a regulator's registers, from its address: millivolts */
#define VRM_ENABLE               4
#define VRM_MODE                 8
#define VRM_MODE_LDO_HPM         7

static vm_offset_t rsc;

/* The RPMh address of a resource by the name the boot loader's command database gave it; 0 = none. */
static uint32_t
rpmh_address(const char *name)
{
	const struct cmd_db_header *db;

	db = (const struct cmd_db_header *)ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);
	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return 0;
	}
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry;

		if (slave->id == 0 || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);
		for (unsigned i = 0; i < slave->count; i++) {
			char id[8] = { 0 };

			strlcpy(id, name, sizeof(id));
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

static bool
rpmh_wait(uint32_t reg, uint32_t mask, uint32_t value, const char *what)
{
	for (int i = 0; i < 2000; i++) {
		if ((rd(TCS(reg)) & mask) == value) {
			return true;
		}
		sleep_us(100);
	}
	printf("umi_touch: RPMh: timeout waiting for %s (register %#x = %#x)\n", what, reg, rd(TCS(reg)));
	return false;
}

/* Send up to three register writes to RPMh and wait until they are done. */
static bool
rpmh_write(const uint32_t *address, const uint32_t *data, unsigned count)
{
	uint32_t irq_enable = rd(TCS(TCS_IRQ_ENABLE));
	bool done;

	/* the slot set must be idle, and switched off the way the hardware wants it: trigger, then enable */
	for (int i = 0; rd(TCS(TCS_STATUS)) == 0; i++) {
		if (i > 2000) {
			printf("umi_touch: RPMh: the command slots stay busy\n");
			return false;
		}
		sleep_us(100);
	}
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_TRIGGER);
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_ENABLE);
	wr(TCS(TCS_IRQ_CLEAR), 1);
	for (unsigned i = 0; i < count; i++) {
		wr(TCS(TCS_CMD_MSGID(i)), MSGID_WRITE_4_BYTES | MSGID_RESPONSE);
		wr(TCS(TCS_CMD_ADDR(i)), address[i]);
		wr(TCS(TCS_CMD_DATA(i)), data[i]);
	}
	wr(TCS(TCS_WAIT_FOR_COMPLETION), (1u << count) - 1);
	wr(TCS(TCS_CMD_ENABLE), (1u << count) - 1);
	wr(TCS(TCS_IRQ_ENABLE), irq_enable | 1);        /* the status bit follows the enable; the interrupt itself stays masked */
	wr(TCS(TCS_CONTROL), TCS_CONTROL_ENABLE);
	wr(TCS(TCS_CONTROL), TCS_CONTROL_ENABLE | TCS_CONTROL_TRIGGER);

	done = rpmh_wait(TCS_IRQ_STATUS, 1, 1, "the request");
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_TRIGGER);
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_ENABLE);
	wr(TCS(TCS_CMD_ENABLE), 0);
	wr(TCS(TCS_WAIT_FOR_COMPLETION), 0);
	wr(TCS(TCS_IRQ_CLEAR), 1);
	wr(TCS(TCS_IRQ_ENABLE), irq_enable);
	return done;
}

/*
 * pm8150a_l1: what fts,bus-reg-name = "vdd" was taken to mean. It is 1.8 V and fixed, and it is
 * still worth asking for -- but the overlay's vdd-supply is PHANDLE_NONE, so this is a guess at
 * the pair's pull-up rail and not a reading of the tree. The rail that is written down is avdd,
 * which panel_supply_on() below raises.
 */
static bool
bus_supply_on(void)
{
	uint32_t base = rpmh_address("ldoc1");
	uint32_t address[3];
	uint32_t data[3];

	if (base == 0 || (base >> 16 & 0xf) != 4) {
		printf("umi_touch: RPMh has no regulator \"ldoc1\" (address %#x)\n", base);
		return false;
	}
	address[0] = base + VRM_VOLTAGE;
	address[1] = base + VRM_ENABLE;
	address[2] = base + VRM_MODE;
	data[0] = 1800;                 /* the node's qcom,init-voltage: 1 800 000 uV */
	data[1] = 1;                    /* on */
	data[2] = VRM_MODE_LDO_HPM;
	printf("umi_touch: RPMh: ldoc1 at %#x, 1800 mV: slot status %#x, control %#x\n", base,
	    rd(TCS(TCS_STATUS)), rd(TCS(TCS_CONTROL)));
	return rpmh_write(address, data, 3);
}

/*
 * pm8150_s4: the rail this part actually runs from.
 *
 * fts@49 in the panel overlay names avdd-supply = <&pm8150_s4> and fts,pwr-reg-name = "avdd";
 * vdd-supply, which fts,bus-reg-name = "vdd" points at, is PHANDLE_NONE -- there is no such
 * supply on this board. The node is rpmh-regulator-smpa4/regulator-pm8150-s4, so the name RPMh
 * knows it by is "smpa4", 0x40000 in the command database, and its qcom,init-voltage is
 * 1 800 000 uV with min and max pinned to the same value.
 *
 * It is an SMPS and not an LDO, so only the voltage and the enable are written: the mode field
 * means something different for each regulator type and the node does not ask for one.
 */
static bool
panel_supply_on(const char *name)
{
	uint32_t base = rpmh_address(name);
	uint32_t address[2];
	uint32_t data[2];

	if (!rsc) {
		rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	}
	if (base == 0) {
		printf("umi_touch: RPMh has no regulator \"%s\" (address %#x)\n", name, base);
		return false;
	}
	address[0] = base + VRM_VOLTAGE;
	address[1] = base + VRM_ENABLE;
	data[0] = 1800;                 /* 1 800 000 uV */
	data[1] = 1;                    /* on */
	printf("umi_touch: RPMh: %s at %#x, 1800 mV (avdd): slot status %#x, control %#x\n", name, base,
	    rd(TCS(TCS_STATUS)), rd(TCS(TCS_CONTROL)));
	return rpmh_write(address, data, 2);
}

/*
 * The same rail, off. Nothing has ever switched it off in this port, and that is the point: this
 * phone is booted with `fastboot boot`, which does not power-cycle the SoC, so whatever state the
 * touch controller was left in by the last run is the state the next run finds it in. A chip that
 * was half way through a byte, or that had its interface register written to something it did not
 * like, stays that way across every run -- and a reset pulse does not clear it, because reset only
 * resets the digital core, not the state machine that is waiting for the rest of a byte.
 *
 * Only the rail being dropped clears that. It is a separate boot argument and not part of the
 * normal sweep because it is the one step here that can take the phone down: if smpa4 feeds
 * anything besides the touch panel, switching it off costs the run.
 */
static bool
panel_supply_off(const char *name)
{
	uint32_t base = rpmh_address(name);
	uint32_t address[1];
	uint32_t data[1];

	if (!rsc) {
		rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	}
	if (base == 0) {
		printf("umi_touch: RPMh has no regulator \"%s\"\n", name);
		return false;
	}
	address[0] = base + VRM_ENABLE;
	data[0] = 0;                    /* off */
	printf("umi_touch: RPMh: %s off\n", name);
	return rpmh_write(address, data, 1);
}

/*
 * Read the id and say what it means, for the line the caller is about to print.
 */
static void
power_id(const char *when)
{
	uint8_t id[2] = { 0, 0 };

	if (!fts_read_registers(FTS_CHIP_ID_ADDR_A, id, 2)) {
		printf("umi_touch: power: %-28s not acknowledged\n", when);
		return;
	}
	printf("umi_touch: power: %-28s id %02x %02x%s\n", when, id[0], id[1],
	    (id[0] == FTS_ID_0_A && id[1] == FTS_ID_1_A) ? "   <- 36 70, the core is running"
	    : (id[0] == FTS_ID_0_B && id[1] == FTS_ID_1_B) ? "   <- 39 6c, the core is running"
	    : (id[0] || id[1]) ? "   <- something, and it is not zero"
	    : "");
}

/*
 * Power the part up the way Android has it, in the order the hardware wants, and read the id after
 * every step so that a run which fails still says which rail was the one.
 *
 * What is known and what is guessed, because the difference matters when reading the log:
 *
 *   ldoc1 (pm8150a_l1)   known. Run 152817 asked RPMh for this rail and nothing else and sda, scl
 *                        and reset went from reading 0 to reading 1 with no pad touched. It is
 *                        fixed at 1.8 V (min = max = init = 1800000 in the base tree) and the
 *                        pair's pull-ups hang from it. With it off, both lines sit at about 0.6 V
 *                        through the chip's esd diodes and every read returns zeros with what
 *                        looks like an acknowledge on every byte -- a line held low *is* one.
 *   gpio69               known to be the switch the fts node names (avdd-supply -> touch_vddio_vreg),
 *                        not known to be enough: with it high and ldoc1 on, the part acknowledges
 *                        its address, and 0x48 does not, so the acknowledge is real; and all 256
 *                        registers still read zero, writes do not stick, and sense-on does nothing.
 *   gpio5, gpio71        the guess, and the reason this function exists. See PIN_VBOOST.
 *
 * The reset pulse comes last, after every rail: a part held in reset while its supplies come up is
 * a part that comes up in an undefined state.
 */
static void
power_up(void)
{
	static const struct {
		unsigned pin;
		const char *what;
		unsigned wait_ms;
	} rails[] = {
		{ PIN_VBOOST, "gpio5, vdd_boost_vreg", 6 },     /* startup-delay-us 4000 */
		{ PIN_VCI,    "gpio71, display_panel_vci", 2 }, /* 3.0 V, boot-on */
	};
	uint32_t v5 = 0;
	/*
	 * ext_5v_boost (gpio12) is the third rail in the same fragment and the last one this driver
	 * has not driven. It stays behind umi_touch_v5=1 because the base tree gives it to
	 * qcom,usb-pdphy as vbus-supply: it is the otg 5 V, and switching it on while this kernel is
	 * talking to the loader over that same usb port is a good way to lose the log that says what
	 * happened. It is worth the risk now that the other two are measured: both were already high
	 * before the driver touched them, so "enable the rail" never changed a thing, and a load
	 * switch whose enable is high proves nothing about the rail behind it. If the display tree is
	 * fed from here, gpio5 and gpio71 being high has meant nothing at all, which would be the one
	 * power explanation the id reads so far cannot see.
	 */
	PE_parse_boot_argn("umi_touch_v5", &v5, sizeof(v5));
	if (v5) {
		printf("umi_touch: power: umi_touch_v5=1 -- driving gpio12 (ext_5v_boost), the usb 5 V\n");
	}

	line_release(pin_sda);
	line_release(pin_scl);
	sleep_ms(2);
	printf("umi_touch: power: as found: sda %u scl %u reset %u int %u\n",
	    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_RESET), gpio_get(PIN_INT));

	rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	printf("umi_touch: power: smpa4 back on (an earlier run turned it off): %s\n",
	    panel_supply_on("smpa4") ? "done" : "refused");
	sleep_ms(10);
	printf("umi_touch: power: ldoc1 (pm8150a_l1, 1.8 V): %s\n",
	    bus_supply_on() ? "done" : "refused");
	sleep_ms(20);
	printf("umi_touch: power: with ldoc1 on: sda %u scl %u reset %u int %u\n",
	    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_RESET), gpio_get(PIN_INT));

	printf("umi_touch: power: gpio%u touch_vddio_vreg %#x/%u -> high\n", PIN_AVDD,
	    rd(GPIO_CFG(PIN_AVDD)), gpio_get(PIN_AVDD));
	gpio_output(PIN_AVDD, true, PULL_NONE, 2);
	sleep_ms(5);
	power_id("vddio switch on");

	for (unsigned i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
		printf("umi_touch: power: %s %#x/%u -> high\n", rails[i].what,
		    rd(GPIO_CFG(rails[i].pin)), gpio_get(rails[i].pin));
		gpio_output(rails[i].pin, true, PULL_NONE, 2);
		sleep_ms(rails[i].wait_ms);
		printf("umi_touch: power: %s is now %#x/%u\n", rails[i].what,
		    rd(GPIO_CFG(rails[i].pin)), gpio_get(rails[i].pin));
		power_id(rails[i].what);
	}

	if (v5) {
		printf("umi_touch: power: gpio%u ext_5v_boost %#x/%u -> high\n", PIN_V5,
		    rd(GPIO_CFG(PIN_V5)), gpio_get(PIN_V5));
		gpio_output(PIN_V5, true, PULL_NONE, 8);
		sleep_ms(10);                   /* enable-ramp-delay 1600 us, then the tree comes up */
		power_id("gpio12, ext_5v_boost");
		sleep_ms(50);
		power_id("gpio12, 50 ms later");
	}

	gpio_output(PIN_RESET, false, PULL_NONE, 2);
	sleep_ms(20);
	gpio_output(PIN_RESET, true, PULL_NONE, 2);
	sleep_ms(settle_ms);
	printf("umi_touch: power: out of reset: sda %u scl %u reset %u int %u\n",
	    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_RESET), gpio_get(PIN_INT));
	power_id("out of reset");
}

static void
umi_touch_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 8000, diag = 0, avdd = 0, swap = 0, one = 0, pp = 0, cd = 0;
	uint32_t sc = 0;
	uint32_t sup = 0;
	uint32_t df = 0;
	uint32_t wt = 0;
	uint32_t sw = 0;
	uint32_t pc = 0;
	uint32_t proto = 0;
	uint32_t bitsweep = 0;
	uint32_t scan8 = 0;
	unsigned idle = 0, idle_looks;
	bool found = false;

	PE_parse_boot_argn("umi_touch_poll_us", &poll_us, sizeof(poll_us));
	PE_parse_boot_argn("umi_touch_diag", &diag, sizeof(diag));
	PE_parse_boot_argn("umi_touch_avdd", &avdd, sizeof(avdd));
	PE_parse_boot_argn("umi_touch_swap", &swap, sizeof(swap));
	PE_parse_boot_argn("umi_touch_flip", &flip, sizeof(flip));
	PE_parse_boot_argn("umi_touch_stage", &stage, sizeof(stage));
	PE_parse_boot_argn("umi_touch_gpio", &one, sizeof(one));
	PE_parse_boot_argn("umi_touch_pushpull", &pp, sizeof(pp));
	pushpull = pp != 0;
	PE_parse_boot_argn("umi_touch_reset", &reset_mode, sizeof(reset_mode));
	PE_parse_boot_argn("umi_touch_bit_us", &bit_us, sizeof(bit_us));
	if (bit_us < 1 || bit_us > 1000) {
		bit_us = 3;
	}
	PE_parse_boot_argn("umi_touch_scan", &sc, sizeof(sc));
	PE_parse_boot_argn("umi_touch_supply", &sup, sizeof(sup));
	PE_parse_boot_argn("umi_touch_diff", &df, sizeof(df));
	PE_parse_boot_argn("umi_touch_wtest", &wt, sizeof(wt));
	PE_parse_boot_argn("umi_touch_settle_ms", &settle_ms, sizeof(settle_ms));
	PE_parse_boot_argn("umi_touch_sweep", &sw, sizeof(sw));
	PE_parse_boot_argn("umi_touch_pcycle", &pc, sizeof(pc));
	PE_parse_boot_argn("umi_touch_proto", &proto, sizeof(proto));
	PE_parse_boot_argn("umi_touch_bitsweep", &bitsweep, sizeof(bitsweep));
	PE_parse_boot_argn("umi_touch_scan8", &scan8, sizeof(scan8));
	if (PE_parse_boot_argn("umi_touch_cmd_db", &cd, sizeof(cd)) && cd) {
		dump_cmd_db();
		thread_terminate(current_thread());
		return;
	}
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 8000;
	}

	tlmm_west = ml_io_map(TLMM_WEST, TLMM_SIZE);
	tlmm_south = ml_io_map(TLMM_SOUTH, TLMM_SIZE);
	tlmm_north = ml_io_map(TLMM_NORTH, TLMM_SIZE);
	if (one) {
		printf("umi_touch: every pin at %#x + 0x1000 * pin, over the tile table\n", one);
		tlmm_one = ml_io_map(one, TLMM_SIZE);
	}
	printf("umi_touch: tlmm west %#x south %#x north %#x; gpio%u..%u and gpio%u are in south\n",
	    (uint32_t)TLMM_WEST, (uint32_t)TLMM_SOUTH, (uint32_t)TLMM_NORTH, PIN_SDA, PIN_INT, PIN_AVDD);
	if (pushpull) {
		printf("umi_touch: driving the high half of the clock instead of waiting for a pull-up\n");
	}

	show_pins("as found");
	/*
	 * umi_touch_supply=1: put the board back the way Android leaves it, in the order the hardware
	 * wants, before anything is measured. power_up() is the whole of it and says what is known and
	 * what is guessed, rail by rail; the short version is that ldoc1 is the 1.8 V this pair hangs
	 * from and gpio69 the switch the fts node names, and the part still does not run on those two
	 * alone, which is why the three rails below the display tree are tried as well.
	 */
	if (sup || proto || bitsweep || scan8) {
		power_up();
	}
	/*
	 * umi_touch_scan8=1: which addresses, anywhere in the space, answer with something other
	 * than zero. See register_scan() -- the id page has been the only range measured, and a
	 * register file in a different window would look exactly like a core that is not running.
	 */
	if (scan8) {
		i2c_bus_recover();
		register_scan();
		thread_terminate(current_thread());
		return;
	}
	/*
	 * umi_touch_proto=1: the chip acknowledges its address and returns zero for every register.
	 * This is the run that decides whether that is a wrong opcode or a core that is not running;
	 * see protocol_scan(). power_up() has just left the chip out of reset with every rail up,
	 * which is the state the scan needs -- pins_for_i2c() would put reset back down.
	 */
	if (proto) {
		i2c_bus_recover();
		printf("umi_touch: the pair before the scan: sda %u scl %u int %u\n",
		    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_INT));
		protocol_scan();
		thread_terminate(current_thread());
		return;
	}
	/*
	 * umi_touch_bitsweep=1: the one variable no run has swept. Every reading this driver has
	 * taken so far was at bit_us=3, and the part has answered its address at that rate and
	 * returned zeros for everything else. A half-period the chip cannot follow -- or one long
	 * enough that its own timeout fires part-way through a transfer -- looks exactly like a
	 * part that is not running, and it is the last thing left that can make a live chip look
	 * dead. So: read the id at each half-period, slowest first, recovering the bus before
	 * each one so that a rate which stalls does not poison the next.
	 */
	if (bitsweep) {
		static const unsigned rates[] = { 100, 50, 25, 12, 6, 3, 1 };

		printf("umi_touch: --- bit rate sweep: the id at each half-period ---\n");
		for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
			uint8_t id[2] = { 0, 0 };
			bool ok;

			bit_us = rates[i];
			i2c_bus_recover();
			ok = fts_read_registers(FTS_CHIP_ID_ADDR_A, id, sizeof(id));
			printf("umi_touch: bit_us %3u: %-4s id %02x %02x%s\n", rates[i],
			    ok ? "read" : "fail", id[0], id[1],
			    (id[0] || id[1]) ? "   <- not zero" : "");
		}
		thread_terminate(current_thread());
		return;
	}
	/*
	 * umi_touch_scan=1: who is on this bus at all, with the reset held one way and then the
	 * other. It runs before anything is decided about the chip, because "no acknowledge at
	 * 0x49" does not say whether the bus is dead or the address is wrong.
	 */
	if (sc) {
		unsigned answered;

		pins_for_i2c();
		gpio_set(PIN_RESET, reset_mode != 0);
		sleep_ms(200);
		answered = scan_bus();
		printf("umi_touch: scan with reset settled %s: %u answered\n",
		    reset_mode ? "high" : "low", answered);
		gpio_set(PIN_RESET, reset_mode == 0);
		sleep_ms(200);
		answered = scan_bus();
		printf("umi_touch: scan with reset the other way: %u answered\n", answered);
		thread_terminate(current_thread());
		return;
	}
	if (stage <= 1) {
		printf("umi_touch: stage %u -- the pins were read and nothing was written; stopping\n", stage);
		thread_terminate(current_thread());
		return;
	}
	if (avdd) {
		printf("umi_touch: gpio%u, the panel I/O rail's enable, is %#x/%u; driving it high\n",
		    PIN_AVDD, rd(GPIO_CFG(PIN_AVDD)), gpio_get(PIN_AVDD));
		gpio_output(PIN_AVDD, true, PULL_NONE, 2);
		sleep_ms(5);
		printf("umi_touch: gpio%u is now %#x/%u\n",
		    PIN_AVDD, rd(GPIO_CFG(PIN_AVDD)), gpio_get(PIN_AVDD));
	}
	idle_looks = 80000 / poll_us;    /* a read every 80 ms even with the line high, in case it is missed */

	if (pc) {
		uint8_t wb = 0x38;              /* ADDR_WARM_BOOT 0x001e, WARM_BOOT_VALUE, ftsHardware.h */
		uint8_t id[2] = { 0, 0 };

		pins_for_i2c();
		i2c_bus_recover();              /* the previous run may have left it mid-byte */
		/*
		 * The order matters and it is the point of this whole step.
		 * The I2C lines are driven low first, while the rail is still up, and they stay low
		 * while it goes down. Driving them
		 * high into an unpowered part is what this driver has been doing for every run of this
		 * session -- the lines go into the chip's esd diodes and back-power it through them, into
		 * a state no reset can reach, because reset only resets the digital core and the state
		 * machine that answers i2c is not in it. A part that acknowledges its address and returns
		 * zeros to everything is what that looks like from the outside.
		 *
		 * pc=1 drops the panel's own io rail (gpio69, touch_vddio_vreg in the overlay), pc=2
		 * drops pm8150_s4, pc=3 does both. The overlay names only the first; the second is here
		 * because "only the first" is how the last several wrong answers happened.
		 */
		printf("umi_touch: a real power cycle: lines low, rail off, rail on, reset\n");
		line_low(pin_sda);
		line_low(pin_scl);
		if (pc & 1) {
			printf("umi_touch: gpio%u off (was %#x/%u)\n", PIN_AVDD,
			    rd(GPIO_CFG(PIN_AVDD)), gpio_get(PIN_AVDD));
			gpio_output(PIN_AVDD, false, PULL_DOWN, 2);
		}
		if (pc & 2) {
			printf("umi_touch: smpa4 off: %s\n", panel_supply_off("smpa4") ? "done" : "refused");
		}
		sleep_ms(200);
		printf("umi_touch: with the rail off: sda %u scl %u reset %u int %u\n",
		    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_RESET), gpio_get(PIN_INT));
		if (pc & 1) {
			gpio_output(PIN_AVDD, true, PULL_NONE, 2);
			printf("umi_touch: gpio%u on: %#x/%u\n", PIN_AVDD,
			    rd(GPIO_CFG(PIN_AVDD)), gpio_get(PIN_AVDD));
		}
		if (pc & 2) {
			printf("umi_touch: smpa4 on: %s\n", panel_supply_on("smpa4") ? "done" : "refused");
		}
		sleep_ms(50);
		/* now let the pair go, and give the part the reset the vendor's power cycle gives it */
		line_release(pin_sda);
		line_release(pin_scl);
		sleep_ms(10);
		gpio_output(PIN_RESET, false, PULL_NONE, 2);
		sleep_ms(20);
		gpio_output(PIN_RESET, true, PULL_NONE, 2);
		sleep_ms(settle_ms);
		printf("umi_touch: after the power cycle: sda %u scl %u reset %u int %u\n",
		    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_RESET), gpio_get(PIN_INT));
		(void)fts_read_verbose(FTS_CHIP_ID_ADDR_A, 2, 1, "id after a power cycle");
		(void)fts_read_verbose(FTS_CHIP_ID_ADDR_B, 2, 1, "id after a power cycle");
		/*
		 * The last thing the vendor's flasher does after burning a firmware is tell the chip to
		 * run it: 0x38 to 0x001e. If this part is sitting in its rom because the application
		 * never started, this is the command that starts it -- and if it is not, 0x001e is a
		 * filter register in the running application and 0x38 only sets a flag in it.
		 */
		printf("umi_touch: warm boot, %#06x <- %02x: %s\n", 0x001e, wb,
		    fts_write_registers(0x001e, &wb, 1) ? "acked" : "not acked");
		sleep_ms(300);
		(void)fts_read_verbose(FTS_CHIP_ID_ADDR_A, 2, 1, "id after a warm boot");
		(void)fts_read_verbose(FTS_CHIP_ID_ADDR_B, 2, 1, "id after a warm boot");
		if (fts_read_registers(FTS_CHIP_ID_ADDR_A, id, 2)) {
			printf("umi_touch: after a power cycle and a warm boot: id %02x %02x\n",
			    id[0], id[1]);
		}
		sample_bits(FTS_CHIP_ID_ADDR_A);
		printf("umi_touch: at the end: sda %u scl %u int %u\n",
		    gpio_get(pin_sda), gpio_get(pin_scl), gpio_get(PIN_INT));
		thread_terminate(current_thread());
		return;
	}
	if (sw) {
		pins_for_i2c();
		gpio_set(PIN_RESET, reset_mode != 0);
		sleep_ms(settle_ms);
		printf("umi_touch: sweeping every protocol variant this chip could want\n");
		sweep();
		thread_terminate(current_thread());
		return;
	}
	if (wt) {
		pins_for_i2c();
		gpio_set(PIN_RESET, reset_mode != 0);
		sleep_ms(200);
		printf("umi_touch: does a write stick?\n");
		write_test();
		thread_terminate(current_thread());
		return;
	}
	if (df) {
		pins_for_i2c();
		gpio_set(PIN_RESET, reset_mode != 0);
		sleep_ms(200);
		printf("umi_touch: reading the map twice to see what moves\n");
		register_diff();
		thread_terminate(current_thread());
		return;
	}
	/*
	 * Which of gpio36/gpio37 is SDA and which is SCL is not written down anywhere, and the
	 * wrong way round is not an error the chip can report: it simply sees no valid start
	 * condition. So both are tried, and only the one that answers is kept.
	 */
	if (stage >= 3) {
		line_test();
	}
	for (int attempt = 0; attempt < 2 && !found; attempt++) {
		unsigned sda = attempt == 0 ? PIN_SDA : PIN_SCL;
		unsigned scl = attempt == 0 ? PIN_SCL : PIN_SDA;

		if (swap) {
			unsigned t = sda;
			sda = scl;
			scl = t;
		}
		found = probe(sda, scl, true);
	}
	if (stage <= 2) {
		printf("umi_touch: stage %u -- the chip %s; stopping\n", stage,
		    found ? "answered" : "did not answer");
		thread_terminate(current_thread());
		return;
	}
	if (!found) {
		printf("umi_touch: no FingerTipS touch screen on I2C %#x\n", I2C_ADDRESS);
		show_pins("no answer");
		thread_terminate(current_thread());
		return;
	}
	show_pins("answering");
	if (diag) {
		register_dump();
	}
	/*
	 * A chip that has just been reset is not scanning the panel and is not driving its interrupt
	 * line, so neither the FIFO nor gpio39 would ever say anything. The vendor driver does the
	 * same two things when it resumes (fts.c: fts_command(FTS_CMD_MS_MT_SENSE_ON) and a write of
	 * IER_ENABLE to IER_ADDR); without them the loop below would poll an idle part forever and
	 * look exactly like a driver that cannot read the chip.
	 */
	{
		uint8_t ier = FTS_IER_ENABLE;
		bool sense_ok = fts_command(FTS_CMD_SENSE_ON);
		bool ier_ok = fts_write_registers(FTS_IER_ADDR, &ier, 1);

		printf("umi_touch: sense on: %s; interrupt enable: %s\n",
		    sense_ok ? "acked" : "not acked", ier_ok ? "acked" : "not acked");
		sleep_ms(20);
	}
	printf("umi_touch: FingerTipS touch screen ready; the interrupt line is looked at every %u us, "
	    "events are read with command %#x\n", poll_us, FTS_CMD_READ_ONE);

	for (unsigned tick = 1;; tick++) {
		if (!gpio_get(PIN_INT)) {
			idle = 0;
			poll_report();
		} else if (++idle >= idle_looks) {
			idle = 0;
			poll_report();
		}
		if (tick % (60 * (1000000 / poll_us)) == 0) {
			printf("umi_touch: so far %u reports in %u looks (%u bad, %u without an acknowledge), "
			    "%u finger positions, the last at %u, %u; interrupt line %s\n", seen.reports, seen.polls,
			    seen.bad, seen.no_ack, seen.touches, seen.last_x, seen.last_y,
			    gpio_get(PIN_INT) ? "high" : "low");
		}
		sleep_us(poll_us);
	}
}

void
umi_touch_init(void)
{
	uint32_t enabled = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("umi_touch", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (kernel_thread_start(umi_touch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
