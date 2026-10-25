/*
 * The touch screen of the Xiaomi Mi 10 (umi): an ST FocalTech "st,fts" controller on the
 * SoC's I2C bus 3 at address 0x49, as a source of touches for /dev/redwood-touch
 * (sys/redwood_input.h). Only with boot-arg umi_touch=1.
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
 * and no engine, and the register protocol is plain -- write the register address, repeat
 * the start, read the bytes -- with none of the FTS 5.x SPI framing (7-byte header, dummy
 * bytes, CRC-16/Kermit) that bsd/dev/redwood_goodix.c has to carry for the same chip
 * family over SPI.
 *
 * Diagnostics, because the first run is the one that has to say what the chip is:
 *   umi_touch_diag=1    after the chip ID, read all 256 registers a byte at a time and print
 *                       what came back: the chip's map, in the loader's log
 *   umi_touch_reg=N     read the touch report from register N instead of 0x01
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
 *   umi_touch_supply=1  ask RPMh for pm8150a_l1 (ldoc1, 1.8 V) before touching the bus
 *   umi_touch_start=0   do not run the 0x55/0xAA start sequence before reading the chip ID
 *   umi_touch_diff=1    read the whole map three times and print only what changes between
 *   umi_touch_wtest=1   write a byte to three registers and read it back, then stop
 *   umi_touch_settle_ms=N  how long the chip gets between the reset and the first read (200)
 *   umi_touch_reg16=1   send a two-byte register address instead of one
 *   umi_touch_sweep=1   try every protocol variant in one boot and print one line each
 *   umi_touch_reset=N   1 (the default) leaves the reset line high after the pulse, 0 leaves it
 *                       low, 2 does not touch it -- one of the three is the chip out of reset
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
 * All five pins this driver uses -- 36, 37, 38, 39 and 69 -- are in TLMM_SOUTH:
 * gpio36 at 0x0f524000, gpio39 at 0x0f527000, gpio69 at 0x0f545000.
 */
#define TLMM_WEST                0x0f100000UL
#define TLMM_SOUTH               0x0f500000UL
#define TLMM_NORTH               0x0f900000UL
#define TLMM_SIZE                0x100000UL

#define GPIO_CFG(n)              (gpio_tile(n) + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)               (gpio_tile(n) + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define PULL_NONE                0
#define PULL_UP                  3

#define PIN_SDA                  36      /* qupv3_se13_i2c_active: "qup13" */
#define PIN_SCL                  37      /* qupv3_se13_i2c_active: "qup13" */
#define PIN_RESET                38      /* fts,reset-gpio; low = in reset */
#define PIN_INT                  39      /* fts,irq-gpio; low = the chip has a report */
#define PIN_AVDD                 69      /* disp_vddio_vreg's enable, active high */

#define I2C_ADDRESS              0x49    /* fts@49 */

#define FTS_REG_CHIP_ID_HIGH     0xa3
#define FTS_REG_CHIP_ID_LOW      0x9f
#define FTS_REG_TOUCH_DATA       0x01

/*
 * Starting the chip: 0xAA written to register 0x55, 12 ms, then the boot ID at register 0x90.
 * bsd/dev/redwood_goodix.c carries the same three numbers for the FT3680 (FTS_CMD_START1,
 * FTS_CMD_START2, FTS_CMD_START_DELAY, FTS_CMD_READ_BOOT_ID) and expects 0x56 0x62 back.
 */
#define FTS_CMD_START_REGISTER   0x55
#define FTS_CMD_START_VALUE      0xaa
#define FTS_CMD_BOOT_ID          0x90
#define FTS_MAX_TOUCHES          10      /* focaltech,max-touch-number */
#define FTS_REPORT_LENGTH        (2 + 6 * FTS_MAX_TOUCHES)

#define FTS_EVENT_DOWN           0
#define FTS_EVENT_UP             1
#define FTS_EVENT_CONTACT        2

static vm_offset_t tlmm_west, tlmm_south, tlmm_north;
static vm_offset_t tlmm_one;            /* umi_touch_gpio: put every pin at this base instead */
static uint32_t stage = 9;              /* umi_touch_stage */
static uint32_t reset_mode = 1;         /* umi_touch_reset: the level the reset sequence settles at */
static bool start_chip = true;          /* umi_touch_start: run the 0x55/0xAA sequence */
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
 * umi_touch_reg16=1 sends two address bytes, 0x00 and then the register, instead of one.
 *
 * It is here because of what this chip does with one: the address byte is acknowledged, the
 * register byte is acknowledged, the data comes back 0x00, and a write into any register is
 * acknowledged and does not stick. That is also exactly what a part expecting a 16-bit register
 * address would do when handed eight -- consume the register byte as its high half, wait for a
 * low half that never comes, and answer nothing. One boot argument tells the two apart.
 */
static bool reg16;
static bool norepeat;                   /* umi_touch_sweep: stop, then start again, instead of repeating */
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

/* A transfer left half finished can hold SDA down; nine clocks let the chip let go of it. */
static void
i2c_bus_recover(void)
{
	int pulses;

	line_release(pin_sda);
	line_release(pin_scl);
	i2c_delay();
	if (gpio_get(pin_sda)) {
		return;                     /* the bus is idle */
	}
	printf("umi_touch: the bus is stuck with SDA low; clocking it free\n");
	for (pulses = 0; pulses < 9 && !gpio_get(pin_sda); pulses++) {
		printf("umi_touch: ... pulse %d: sda %u scl %#x\n", pulses,
		    gpio_get(pin_sda), rd(GPIO_CFG(pin_scl)));
		line_low(pin_scl);
		i2c_delay();
		line_release(pin_scl);
		i2c_delay();
	}
	printf("umi_touch: ... %d pulse(s), sda now %u\n", pulses, gpio_get(pin_sda));
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

/* Write the register, repeat the start, read the bytes: the whole of I2C register access. */static bool
i2c_read_registers(uint8_t reg, uint8_t *data, uint32_t length)
{
	bool ok = false;

	i2c_start();
	if (i2c_write_byte(bus_address << 1) && (reg16 ? i2c_write_byte(0x00) : true) &&
	    i2c_write_byte(reg)) {
		if (norepeat) {
			i2c_stop();
		}
		i2c_start();                                   /* repeated start, unless asked for a stop */
		if (i2c_write_byte(bus_address << 1 | 1)) {
			for (uint32_t i = 0; i < length; i++) {
				data[i] = i2c_read_byte(i + 1 < length);
			}
			ok = true;
		}
	}
	i2c_stop();
	return ok;
}

static bool
i2c_write_registers(uint8_t reg, const uint8_t *data, uint32_t length)
{
	bool ok;

	i2c_start();
	ok = i2c_write_byte(bus_address << 1) && (reg16 ? i2c_write_byte(0x00) : true) &&
	    i2c_write_byte(reg);
	for (uint32_t i = 0; ok && i < length; i++) {
		ok = i2c_write_byte(data[i]);
	}
	i2c_stop();
	return ok;
}

/* ---- the chip ----------------------------------------------------------------------------------- */

static uint32_t touch_register = FTS_REG_TOUCH_DATA;
static uint32_t flip;              /* boot-arg umi_touch_flip */

static struct {
	uint32_t polls, reports, touches, bad, no_ack, last_x, last_y;
} seen;

static struct {
	bool down, seen;
	uint32_t x, y;
} contact[FTS_MAX_TOUCHES];

/*
 * The report at register 0x01, in the layout the vendor's focaltech_core.c reads it (the
 * same offsets bsd/dev/redwood_goodix.c uses for the SPI variant of this chip family):
 *   report[0]            gesture (0 outside gesture mode)
 *   report[1]            (chip type << 4) | finger count
 *   report[2 + 6*i + 0]  E_XH:  [7:6] event (0 down, 1 up, 2 contact), [3:0] x bits 14..11
 *   report[2 + 6*i + 1]  XL:    x bits 10..3
 *   report[2 + 6*i + 2]  ID_YH: [7:4] finger id, [3:0] y bits 14..11
 *   report[2 + 6*i + 3]  YL:    y bits 10..3
 *   report[2 + 6*i + 4]  PRE:   [7:5] x bits 2..0, [4:2] y bits 2..0
 *   report[2 + 6*i + 5]  AREA:  touch area
 * Both axes are 15 bits; the panel is 1080 x 2340.
 */
#define PANEL_X 1080
#define PANEL_Y 2340

static void
report_touch(uint32_t id, uint32_t event, uint32_t x, uint32_t y)
{
	static unsigned shown;

	if (id >= FTS_MAX_TOUCHES) {
		return;
	}
	if (event == FTS_EVENT_UP) {
		contact[id].seen = true;
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
	contact[id].seen = true;
	if (!contact[id].down) {
		contact[id].down = true;
		redwood_touch_post(REDWOOD_TOUCH_DOWN, id, x, y);
	} else if (contact[id].x != x || contact[id].y != y) {
		redwood_touch_post(REDWOOD_TOUCH_MOVE, id, x, y);
	}
	contact[id].x = x;
	contact[id].y = y;
	if (shown++ < 8) {
		printf("umi_touch: finger %u %s at %u, %u\n", id,
		    event == FTS_EVENT_DOWN ? "down" : "move", x, y);
	}
}

static void
parse_report(const uint8_t *report)
{
	/*
	 * report[1]'s low nibble is how many fingers the chip is reporting. It is not decoration:
	 * a report of all zeros is what a bus that reads back nothing looks like, and taking its
	 * ten empty slots at face value -- id 0, event 0 (down), x and y zero -- posted a finger
	 * pressed in the top-left corner every 16 ms for as long as the run lasted.
	 */
	uint32_t fingers = report[1] & 0x0f;

	for (uint32_t i = 0; i < FTS_MAX_TOUCHES; i++) {
		contact[i].seen = false;
	}
	if (fingers == 0 || fingers > FTS_MAX_TOUCHES) {
		for (uint32_t i = 0; i < FTS_MAX_TOUCHES; i++) {
			if (contact[i].down) {
				contact[i].down = false;
				redwood_touch_post(REDWOOD_TOUCH_UP, i, contact[i].x, contact[i].y);
			}
		}
		return;
	}
	for (uint32_t i = 0; i < fingers; i++) {
		const uint8_t *t = report + 2 + 6 * i;
		uint32_t id = (t[2] >> 4) & 0x0f;
		uint32_t event, x, y;

		if (id >= FTS_MAX_TOUCHES) {
			break;                  /* no more valid slots in this report */
		}
		event = (t[0] >> 6) & 0x3;
		x = ((uint32_t)(t[0] & 0x0f) << 11) | ((uint32_t)t[1] << 3) | ((uint32_t)t[4] >> 5);
		y = ((uint32_t)(t[2] & 0x0f) << 11) | ((uint32_t)t[3] << 3) | (((uint32_t)t[4] & 0x1c) >> 2);
		report_touch(id, event, x, y);
	}
	/* a finger that is no longer in the report has lifted */
	for (uint32_t i = 0; i < FTS_MAX_TOUCHES; i++) {
		if (!contact[i].seen && contact[i].down) {
			contact[i].down = false;
			redwood_touch_post(REDWOOD_TOUCH_UP, i, contact[i].x, contact[i].y);
		}
	}
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

static bool
read_chip_id(uint8_t *high, uint8_t *low)
{
	return i2c_read_registers(FTS_REG_CHIP_ID_HIGH, high, 1) &&
	    i2c_read_registers(FTS_REG_CHIP_ID_LOW, low, 1);
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
	 * The chip does not answer at its ID registers until it has been started. Focaltech's own
	 * sequence -- the one bsd/dev/redwood_goodix.c uses for the FT3680 over SPI -- is a write of
	 * 0xAA to register 0x55, a 12 ms wait, then two bytes from register 0x90: the boot ID, which
	 * is 0x56 0x62 for that family. A chip that has not been started answers there and nowhere
	 * else, which is what the 256-register map looks like now that the bus is powered: four
	 * non-zero bytes and 252 zeros.
	 */
	if (start_chip) {
		static const uint8_t aa = FTS_CMD_START_VALUE;
		uint8_t boot[2] = { 0, 0 };

		if (i2c_write_registers(FTS_CMD_START_REGISTER, &aa, 1)) {
			sleep_ms(12);
			if (i2c_read_registers(FTS_CMD_BOOT_ID, boot, 2)) {
				printf("umi_touch: boot ID at %#x: %02x %02x%s\n", FTS_CMD_BOOT_ID, boot[0], boot[1],
				    (boot[0] == 0x56 && boot[1] == 0x62) ? " -- the Focaltech ROM answered" : "");
			} else {
				printf("umi_touch: no answer at the boot ID, %#x\n", FTS_CMD_BOOT_ID);
			}
		} else {
			printf("umi_touch: could not write %#x to %#x to start the chip\n",
			    FTS_CMD_START_VALUE, FTS_CMD_START_REGISTER);
		}
	}

	for (int attempt = 0; attempt < 5; attempt++) {
		if (verbose) {
			trace("asking who it is");
		}
		if (read_chip_id(&high, &low)) {
			if (high != 0xff || low != 0xff) {
				if (verbose) {
					printf("umi_touch: chip at %#x on sda gpio%u / scl gpio%u: id %02x %02x (after %d attempt(s))\n",
					    I2C_ADDRESS, pin_sda, pin_scl, high, low, attempt + 1);
				}
				return true;
			}
		} else {
			seen.no_ack++;
			if (verbose) {
				printf("umi_touch: ... attempt %d: no acknowledge\n", attempt + 1);
			}
		}
		sleep_ms(50);
	}
	if (verbose) {
		printf("umi_touch: no answer at %#x on sda gpio%u / scl gpio%u (last read %02x %02x)\n",
		    I2C_ADDRESS, pin_sda, pin_scl, high, low);
	}
	return false;
}

static void
register_dump(void)
{
	uint8_t line[16];

	printf("umi_touch: registers at %#x (sda gpio%u, scl gpio%u):\n", I2C_ADDRESS, pin_sda, pin_scl);
	for (unsigned base = 0; base < 256; base += 16) {
		for (unsigned i = 0; i < 16; i++) {
			line[i] = 0xff;
			if (!i2c_read_registers((uint8_t)(base + i), &line[i], 1)) {
				line[i] = 0xfe;    /* no acknowledge */
			}
		}
		printf("umi_touch: %02x: %02x %02x %02x %02x %02x %02x %02x %02x "
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
		if (!i2c_read_registers((uint8_t)i, &before[i], 1)) {
			before[i] = 0xfe;
		}
	}
	for (int round = 1; round <= 3; round++) {
		unsigned changed = 0;

		sleep_ms(200);
		for (unsigned i = 0; i < 256; i++) {
			after[i] = 0xff;
			if (!i2c_read_registers((uint8_t)i, &after[i], 1)) {
				after[i] = 0xfe;
			}
		}
		for (unsigned i = 0; i < 256; i++) {
			if (before[i] != after[i]) {
				printf("umi_touch:   %02x: %02x -> %02x\n", i, before[i], after[i]);
				changed++;
				before[i] = after[i];
			}
		}
		printf("umi_touch: round %d: %u register(s) moved in 200 ms\n", round, changed);
	}
}

/*
 * Do writes stick? Reading tells us what the chip says about itself; writing a byte and reading it
 * back tells us whether anything behind the I2C slave is alive at all. A part that acknowledges its
 * address, answers 0 for 252 of its 256 registers and ignores a write has its bus interface up and
 * nothing behind it -- which is what a controller that has not been given its application firmware
 * looks like. Each register is put back the way it was found.
 */
static void
write_test(void)
{
	static const uint8_t where[3] = { 0x50, 0xa5, 0x20 };

	for (unsigned i = 0; i < sizeof(where); i++) {
		uint8_t old = 0xff, now = 0xff, back = 0xff, five_a = 0x5a;
		bool old_ok, wrote;

		old_ok = i2c_read_registers(where[i], &old, 1);
		wrote = i2c_write_registers(where[i], &five_a, 1);
		(void)i2c_read_registers(where[i], &now, 1);
		if (old_ok) {
			(void)i2c_write_registers(where[i], &old, 1);
			(void)i2c_read_registers(where[i], &back, 1);
		}
		printf("umi_touch: %#04x: read %02x; write 5a %s; reads %02x; put back %02x\n",
		    where[i], old, wrote ? "acked" : "not acked", now, back);
	}
}

/*
 * Every protocol variant this chip could want, tried in one boot, printed one line each.
 *
 * One experiment per build was the wrong shape: each answer cost a rebuild, a download and a
 * reboot, and the phone sat in the loader between them. Nothing here is destructive and every
 * combination takes milliseconds, so they all go in one run: eight or sixteen bit register
 * addressing, repeated start or stop-then-start, the two addresses the panel overlays name, and
 * with or without Focaltech's 0x55/0xAA start sequence. What a working variant looks like is the
 * chip ID at fts,x-max's driver registers -- 0x56 0x62 for the FT3680 family -- or at least a
 * register map with structure in it instead of four bytes.
 */
static void
sweep(void)
{
	static const uint8_t addresses[2] = { 0x49, 0x38 };

	/*
	 * Every combination has to fit in about a second: this kernel reaches bsd_init and panics on
	 * the missing launchd roughly 1.4 s after the driver starts, and a 256-register dump per
	 * combination spends that budget on two of sixteen. So a combination costs seven transfers --
	 * boot ID, both ID bytes, four registers that were non-zero, and a write-and-read-back -- and
	 * only a combination that looks interesting pays for the whole map.
	 */
	for (unsigned a = 0; a < 2; a++) {
		for (unsigned r16 = 0; r16 < 2; r16++) {
			for (unsigned nr = 0; nr < 2; nr++) {
				for (unsigned st = 0; st < 2; st++) {
					uint8_t idh = 0, idl = 0, boot[2] = { 0, 0 };
					uint8_t probe_old = 0, probe_now = 0, five_a = 0x5a;
					uint8_t r86 = 0, ra6 = 0, ra8 = 0, rfa = 0;
					bool stuck, interesting;

					bus_address = addresses[a];
					reg16 = r16 != 0;
					norepeat = nr != 0;
					if (st) {
						uint8_t aa = FTS_CMD_START_VALUE;

						(void)i2c_write_registers(FTS_CMD_START_REGISTER, &aa, 1);
						sleep_ms(12);
					}
					(void)i2c_read_registers(FTS_CMD_BOOT_ID, boot, 2);
					(void)i2c_read_registers(FTS_REG_CHIP_ID_HIGH, &idh, 1);
					(void)i2c_read_registers(FTS_REG_CHIP_ID_LOW, &idl, 1);
					(void)i2c_read_registers(0x86, &r86, 1);
					(void)i2c_read_registers(0xa6, &ra6, 1);
					(void)i2c_read_registers(0xa8, &ra8, 1);
					(void)i2c_read_registers(0xfa, &rfa, 1);
					(void)i2c_read_registers(0x50, &probe_old, 1);
					(void)i2c_write_registers(0x50, &five_a, 1);
					(void)i2c_read_registers(0x50, &probe_now, 1);
					(void)i2c_write_registers(0x50, &probe_old, 1);
					stuck = probe_now == five_a;
					printf("umi_touch: sweep %#04x r16=%u nr=%u st=%u: boot %02x %02x id %02x %02x"
					    " 86=%02x a6=%02x a8=%02x fa=%02x write %s\n",
					    addresses[a], r16, nr, st, boot[0], boot[1], idh, idl,
					    r86, ra6, ra8, rfa, stuck ? "STUCK" : "ignored");
					interesting = stuck || idh != 0x00 || idl != 0x00 ||
					    (boot[0] != 0x00 && boot[0] != 0xff);
					if (interesting) {
						printf("umi_touch: that one is worth the map://n");
						register_dump();
					}
				}
			}
		}
	}
}

static void
poll_report(void)
{
	static uint8_t report[FTS_REPORT_LENGTH];
	static unsigned bad, shown_raw;

	seen.polls++;
	if (!i2c_read_registers((uint8_t)touch_register, report, sizeof(report))) {
		seen.bad++;
		if (bad++ < 8) {
			printf("umi_touch: reading register %#x failed\n", touch_register);
		}
		return;
	}
	if (shown_raw < 4) {
		shown_raw++;
		printf("umi_touch: report at %#x: %02x %02x %02x %02x %02x %02x %02x %02x "
		    "%02x %02x %02x %02x %02x %02x %02x %02x ...\n", touch_register,
		    report[0], report[1], report[2], report[3], report[4], report[5], report[6], report[7],
		    report[8], report[9], report[10], report[11], report[12], report[13], report[14], report[15]);
	}
	seen.reports++;
	parse_report(report);
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

/* pm8150a_l1, the pair's bus supply: 1.8 V, fixed, and per the overlays nothing else uses it. */
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

static void
umi_touch_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 8000, diag = 0, avdd = 0, swap = 0, reg = 0, one = 0, pp = 0, cd = 0;
	uint32_t sc = 0;
	uint32_t sup = 0;
	uint32_t df = 0;
	uint32_t wt = 0;
	uint32_t sw = 0;
	unsigned idle = 0, idle_looks;
	bool found = false;

	PE_parse_boot_argn("umi_touch_poll_us", &poll_us, sizeof(poll_us));
	PE_parse_boot_argn("umi_touch_diag", &diag, sizeof(diag));
	PE_parse_boot_argn("umi_touch_avdd", &avdd, sizeof(avdd));
	PE_parse_boot_argn("umi_touch_swap", &swap, sizeof(swap));
	PE_parse_boot_argn("umi_touch_reg", &reg, sizeof(reg));
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
	{
		uint32_t start = 1;

		PE_parse_boot_argn("umi_touch_start", &start, sizeof(start));
		start_chip = start != 0;
	}
	PE_parse_boot_argn("umi_touch_diff", &df, sizeof(df));
	PE_parse_boot_argn("umi_touch_wtest", &wt, sizeof(wt));
	PE_parse_boot_argn("umi_touch_settle_ms", &settle_ms, sizeof(settle_ms));
	PE_parse_boot_argn("umi_touch_sweep", &sw, sizeof(sw));
	{
		uint32_t r16 = 0;

		PE_parse_boot_argn("umi_touch_reg16", &r16, sizeof(r16));
		reg16 = r16 != 0;
	}
	if (PE_parse_boot_argn("umi_touch_cmd_db", &cd, sizeof(cd)) && cd) {
		dump_cmd_db();
		thread_terminate(current_thread());
		return;
	}
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 8000;
	}
	if (reg) {
		touch_register = reg;
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
	 * umi_touch_supply=1: ask RPMh for pm8150a_l1 before anything else, because without that rail
	 * there is no high level on the pair and no device to talk to.
	 */
	if (sup) {
		rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
		printf("umi_touch: asking RPMh for the bus supply: %s\n",
		    bus_supply_on() ? "done" : "refused or timed out");
		sleep_ms(20);
		show_pins("with the bus supply asked for");
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
		printf("umi_touch: no FocalTech touch screen on I2C %#x\n", I2C_ADDRESS);
		show_pins("no answer");
		thread_terminate(current_thread());
		return;
	}
	show_pins("answering");
	if (diag) {
		register_dump();
	}
	printf("umi_touch: FocalTech touch screen ready; the interrupt line is looked at every %u us, "
	    "the report is read from register %#x\n", poll_us, touch_register);

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
