// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Himax hx83112b touchscreens
 *
 * Copyright (C) 2022 Job Noorman <job@noorman.info>
 *
 * HX83100A support
 * Copyright (C) 2024 Felix Kaechele <felix@kaechele.ca>
 *
 * This code is based on "Himax Android Driver Sample Code for QCT platform":
 *
 * Copyright (C) 2017 Himax Corporation.
 */

#include <drm/drm_panel.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>

#define HIMAX_MAX_POINTS		10

#define HIMAX_AHB_ADDR_BYTE_0			0x00
#define HIMAX_AHB_ADDR_RDATA_BYTE_0		0x08
#define HIMAX_AHB_ADDR_ACCESS_DIRECTION		0x0c
#define HIMAX_AHB_ADDR_INCR4			0x0d
#define HIMAX_AHB_ADDR_CONTI			0x13
#define HIMAX_AHB_ADDR_EVENT_STACK		0x30

#define HIMAX_AHB_CMD_ACCESS_DIRECTION_READ	0x00
#define HIMAX_AHB_CMD_INCR4			0x10
#define HIMAX_AHB_CMD_CONTI			0x31

#define HIMAX_REG_ADDR_ICID			0x900000d0

#define HX83100A_REG_FW_EVENT_STACK		0x90060000

#define HIMAX_INVALID_COORD		0xffff

/*
 * Time the controller needs on its supplies before it answers the bus. Measured
 * on a Fairphone 3 (hx83112b): with the panel powered down, a probe that
 * enabled the rails and reset the part straight away had its product-id read
 * NACKed three times out of three, and succeeded first time once the rails had
 * been up for a while. 20 ms is chosen, not measured as a minimum.
 */
#define HIMAX_POWER_ON_DELAY_MS		20

/*
 * Firmware configuration words, reached through the AHB window like any other
 * register. The idle-mode word and its switch bit are those the vendor driver
 * in Fairphone's published FP3 kernel toggles in himax_idle_mode()
 * (drivers/input/touchscreen/hxchipset83112b/himax_ic.c): it writes 0x1f to
 * the low byte to enable idle mode and 0x17 to disable it, so bit 3 is the
 * switch and the other bits are left as the firmware set them.
 */
#define HIMAX_REG_FW_IDLE_MODE		0x10007088
#define HIMAX_FW_IDLE_MODE_ENABLE	BIT(3)

/*
 * After a reset the firmware reloads its configuration from flash and
 * overwrites the idle-mode word with the stored default. Measured on a
 * Fairphone 3: a value written a millisecond after the reset was still there
 * 10 ms after probe returned and gone by 50 ms. The vendor driver waits 20 ms
 * after releasing the reset line before it touches the part; 100 ms leaves
 * twice the measured margin.
 */
#define HIMAX_FW_RELOAD_MS		100

/*
 * The charger-mode word is what the same vendor driver writes in
 * himax_usb_detect_set() whenever the battery reports a charger: one magic
 * value with a charger present, another without. The firmware defaults to
 * neither.
 */
#define HIMAX_REG_FW_CHARGER_MODE	0x10007f38
#define HIMAX_FW_CHARGER_MODE_ON	0xa55aa55a
#define HIMAX_FW_CHARGER_MODE_OFF	0x77887788

struct himax_event_point {
	__be16 x;
	__be16 y;
} __packed;

struct himax_event {
	struct himax_event_point points[HIMAX_MAX_POINTS];
	u8 majors[HIMAX_MAX_POINTS];
	u8 pad0[2];
	u8 num_points;
	u8 pad1[2];
	u8 checksum_fix;
} __packed;

static_assert(sizeof(struct himax_event) == 56);

struct himax_ts_data;
struct himax_chip {
	u32 id;
	int (*check_id)(struct himax_ts_data *ts);
	int (*read_events)(struct himax_ts_data *ts, struct himax_event *event,
			   size_t length);
};

struct himax_ts_data {
	const struct himax_chip *chip;
	struct gpio_desc *gpiod_rst;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct touchscreen_properties props;
	unsigned int read_errors;
	bool fw_config_dirty;
	unsigned long reset_jiffies;
	struct notifier_block psy_nb;
	struct drm_panel_follower panel_follower;
	bool is_panel_follower;
};

/*
 * iovcc is the rail the display half of the same controller also takes; vdda is
 * the analog rail. Both are optional so that boards which do not describe them
 * keep working on the dummy regulator.
 */
static const char * const himax_supplies[] = { "iovcc", "vdda" };

static const struct regmap_config himax_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int himax_bus_enable_burst(struct himax_ts_data *ts)
{
	int error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_CONTI,
			     HIMAX_AHB_CMD_CONTI);
	if (error)
		return error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_INCR4,
			     HIMAX_AHB_CMD_INCR4);
	if (error)
		return error;

	return 0;
}

static int himax_bus_read(struct himax_ts_data *ts, u32 address, void *dst,
			  size_t length)
{
	int error;

	if (length > 4) {
		error = himax_bus_enable_burst(ts);
		if (error)
			return error;
	}

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_BYTE_0, address);
	if (error)
		return error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_ACCESS_DIRECTION,
			     HIMAX_AHB_CMD_ACCESS_DIRECTION_READ);
	if (error)
		return error;

	if (length > 4)
		error = regmap_noinc_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0,
					  dst, length);
	else
		error = regmap_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0,
				    dst);
	if (error)
		return error;

	return 0;
}

/*
 * The AHB command registers are one byte wide and adjacent. The regmap is
 * 32-bit, so a regmap_write() to one of them also writes three bytes of zero
 * into its neighbours. Ahead of an event-stack read that is harmless; measured
 * on a Fairphone 3, a firmware-word read issued after it comes back as its
 * first byte repeated. The firmware words are therefore reached with the
 * command registers written one byte at a time, which is also the form the
 * vendor driver uses.
 */
static int himax_ahb_command(struct himax_ts_data *ts, u8 reg, u8 cmd)
{
	return i2c_smbus_write_byte_data(ts->client, reg, cmd);
}

static int himax_fw_window(struct himax_ts_data *ts)
{
	int error;

	error = himax_ahb_command(ts, HIMAX_AHB_ADDR_CONTI, HIMAX_AHB_CMD_CONTI);
	if (error)
		return error;

	return himax_ahb_command(ts, HIMAX_AHB_ADDR_INCR4, HIMAX_AHB_CMD_INCR4);
}

static int himax_read_fw_word(struct himax_ts_data *ts, u32 address, u32 *val)
{
	int error;

	error = himax_fw_window(ts);
	if (error)
		return error;

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_BYTE_0, address);
	if (error)
		return error;

	error = himax_ahb_command(ts, HIMAX_AHB_ADDR_ACCESS_DIRECTION,
				  HIMAX_AHB_CMD_ACCESS_DIRECTION_READ);
	if (error)
		return error;

	return regmap_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0, val);
}

/*
 * A write through the AHB window is a single transfer to the address register:
 * the target address followed by the data, both little-endian.
 */
static int himax_write_fw_word(struct himax_ts_data *ts, u32 address, u32 val)
{
	u32 words[2] = { address, val };
	int error;

	error = himax_fw_window(ts);
	if (error)
		return error;

	return regmap_bulk_write(ts->regmap, HIMAX_AHB_ADDR_BYTE_0, words,
				 ARRAY_SIZE(words));
}

static int himax_set_fw_idle_mode(struct himax_ts_data *ts, bool enable)
{
	u32 val, want;
	int error;

	error = himax_read_fw_word(ts, HIMAX_REG_FW_IDLE_MODE, &val);
	if (error)
		return error;

	want = enable ? val | HIMAX_FW_IDLE_MODE_ENABLE :
			val & ~HIMAX_FW_IDLE_MODE_ENABLE;
	if (want == val)
		return 0;

	error = himax_write_fw_word(ts, HIMAX_REG_FW_IDLE_MODE, want);
	if (error)
		return error;

	/* A refused write is silent; only the read-back proves it took. */
	error = himax_read_fw_word(ts, HIMAX_REG_FW_IDLE_MODE, &val);
	if (error)
		return error;

	return (val ^ want) & HIMAX_FW_IDLE_MODE_ENABLE ? -EIO : 0;
}

static int himax_set_fw_charger_mode(struct himax_ts_data *ts, bool present)
{
	u32 want = present ? HIMAX_FW_CHARGER_MODE_ON : HIMAX_FW_CHARGER_MODE_OFF;
	u32 val;
	int error;

	error = himax_read_fw_word(ts, HIMAX_REG_FW_CHARGER_MODE, &val);
	if (error)
		return error;

	if (val == want)
		return 0;

	error = himax_write_fw_word(ts, HIMAX_REG_FW_CHARGER_MODE, want);
	if (error)
		return error;

	error = himax_read_fw_word(ts, HIMAX_REG_FW_CHARGER_MODE, &val);
	if (error)
		return error;

	return val == want ? 0 : -EIO;
}

/*
 * The vendor driver re-sends its settings after every reset, so this runs after
 * the reset in probe and after the one the interrupt handler issues on a wedged
 * controller. It talks to the part and therefore only runs where the part is
 * known to be awake: right after a reset, or on the interrupt thread.
 *
 * With idle mode enabled the firmware lowers its scan rate after a short period
 * without a touch. On a Fairphone 3 taps go missing after such a pause, with
 * no interrupt raised for them at all; keeping the part out of idle mode is
 * the one state the vendor driver changes that the firmware defaults the other
 * way.
 */
static void himax_apply_fw_config(struct himax_ts_data *ts)
{
	bool dirty = false;
	int error;

	/* Written now, it would be overwritten by the firmware's reload. */
	if (time_before(jiffies, ts->reset_jiffies +
				 msecs_to_jiffies(HIMAX_FW_RELOAD_MS))) {
		WRITE_ONCE(ts->fw_config_dirty, true);
		return;
	}

	error = himax_set_fw_idle_mode(ts, false);
	if (error) {
		dev_warn_ratelimited(&ts->client->dev,
				     "Failed to disable idle mode: %d\n", error);
		dirty = true;
	}

	/*
	 * Without a power-supply class there is nothing to ask, and the part is
	 * left as the vendor stack leaves it with the cable out.
	 */
	error = power_supply_is_system_supplied();
	if (error >= 0) {
		error = himax_set_fw_charger_mode(ts, error > 0);
		if (error) {
			dev_warn_ratelimited(&ts->client->dev,
					     "Failed to set charger mode: %d\n",
					     error);
			dirty = true;
		}
	}

	WRITE_ONCE(ts->fw_config_dirty, dirty);
}

/*
 * A charger coming or going is applied from the interrupt thread, where the
 * part is known to answer the bus; here only the fact is recorded. Battery
 * events are the frequent ones and carry nothing for the controller.
 */
static int himax_psy_notifier(struct notifier_block *nb, unsigned long event,
			      void *data)
{
	struct himax_ts_data *ts = container_of(nb, struct himax_ts_data, psy_nb);
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED &&
	    psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		WRITE_ONCE(ts->fw_config_dirty, true);

	return NOTIFY_OK;
}

static void himax_psy_unreg_notifier(void *data)
{
	power_supply_unreg_notifier(data);
}

static void himax_reset(struct himax_ts_data *ts)
{
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);

	/* Delay copied from downstream driver */
	msleep(20);
	gpiod_set_value_cansleep(ts->gpiod_rst, 0);

	/*
	 * The downstream driver doesn't contain this delay but is seems safer
	 * to include it. The range is just a guess that seems to work well.
	 */
	usleep_range(1000, 1100);
	ts->reset_jiffies = jiffies;
}

static int himax_read_product_id(struct himax_ts_data *ts, u32 *product_id)
{
	int error;

	error = himax_bus_read(ts, HIMAX_REG_ADDR_ICID, product_id,
			       sizeof(*product_id));
	if (error)
		return error;

	*product_id >>= 8;
	return 0;
}

static int himax_check_product_id(struct himax_ts_data *ts)
{
	int error;
	u32 product_id;

	error = himax_read_product_id(ts, &product_id);
	if (error)
		return error;

	dev_dbg(&ts->client->dev, "Product id: %x\n", product_id);

	if (product_id == ts->chip->id)
		return 0;

	dev_err(&ts->client->dev, "Unknown product id: %x\n",
		product_id);
	return -EINVAL;
}

static int himax_input_register(struct himax_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input_dev->name = "Himax Touchscreen";

	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 200, 0, 0);

	touchscreen_parse_properties(ts->input_dev, true, &ts->props);

	error = input_mt_init_slots(ts->input_dev, HIMAX_MAX_POINTS,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to initialize MT slots: %d\n", error);
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static u8 himax_event_get_num_points(const struct himax_event *event)
{
	if (event->num_points == 0xff)
		return 0;
	else
		return event->num_points & 0x0f;
}

static bool himax_process_event_point(struct himax_ts_data *ts,
				      const struct himax_event *event,
				      int point_index)
{
	const struct himax_event_point *point = &event->points[point_index];
	u16 x = be16_to_cpu(point->x);
	u16 y = be16_to_cpu(point->y);
	u8 w = event->majors[point_index];

	if (x == HIMAX_INVALID_COORD || y == HIMAX_INVALID_COORD)
		return false;

	input_mt_slot(ts->input_dev, point_index);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input_dev, &ts->props, x, y, true);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	return true;
}

static void himax_process_event(struct himax_ts_data *ts,
				const struct himax_event *event)
{
	int i;
	int num_points_left = himax_event_get_num_points(event);

	for (i = 0; i < HIMAX_MAX_POINTS && num_points_left > 0; i++) {
		if (himax_process_event_point(ts, event, i))
			num_points_left--;
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

static bool himax_verify_checksum(struct himax_ts_data *ts,
				  const struct himax_event *event)
{
	u8 *data = (u8 *)event;
	int i;
	u16 checksum = 0;

	for (i = 0; i < sizeof(*event); i++)
		checksum += data[i];

	if ((checksum & 0x00ff) != 0) {
		dev_err(&ts->client->dev, "Wrong event checksum: %04x\n",
			checksum);
		return false;
	}

	return true;
}

static int himax_read_events(struct himax_ts_data *ts,
			     struct himax_event *event, size_t length)
{
	return regmap_raw_read(ts->regmap, HIMAX_AHB_ADDR_EVENT_STACK, event,
			       length);
}

static int hx83100a_read_events(struct himax_ts_data *ts,
				struct himax_event *event, size_t length)
{
	return himax_bus_read(ts, HX83100A_REG_FW_EVENT_STACK, event, length);
};

/*
 * The controller intermittently fails to answer a read of the event stack, and
 * a single failure loses the touch that caused the interrupt - including, when
 * it happens mid-gesture, the release, so userspace is left holding a button
 * down until something rebinds the driver. The vendor driver retries every
 * register access, and ak7375 on the same board was given the same treatment
 * for the same -ETIMEDOUT signature. A handful of retries costs nothing when
 * the bus is healthy and covers a transient failure.
 */
#define HIMAX_READ_RETRIES	3

static int himax_handle_input(struct himax_ts_data *ts)
{
	int error, tries = HIMAX_READ_RETRIES;
	struct himax_event event;

	do {
		error = ts->chip->read_events(ts, &event, sizeof(event));
	} while (error && --tries);

	if (error) {
		/*
		 * Rate-limited: when the controller wedges it asserts its
		 * interrupt continuously, and an unlimited dev_err() then
		 * writes over a hundred lines a second to the log for as long
		 * as the wedge lasts - measured at ~158/s for three minutes,
		 * which is a fault of its own on a phone.
		 */
		dev_err_ratelimited(&ts->client->dev,
				    "Failed to read input event: %d\n", error);
		return error;
	}

	/*
	 * Only process the current event when it has a valid checksum but
	 * don't consider it a fatal error when it doesn't.
	 */
	if (!himax_verify_checksum(ts, &event))
		return 0;

	/*
	 * An all-zero event is not something this controller reports. Its idle
	 * frame is 0xff - himax_event_get_num_points() special-cases
	 * num_points == 0xff, and an empty slot is HIMAX_INVALID_COORD, which
	 * is 0xffff. Yet a buffer of zeroes passes himax_verify_checksum(),
	 * because that sums the bytes and requires the low byte of the sum to
	 * be zero, which an empty buffer satisfies trivially. It then yields
	 * zero points, so himax_process_event() reports nothing, and the input
	 * core suppresses the redundant SYN_REPORT - leaving no frame, no
	 * error and no trace of any kind.
	 *
	 * That is the shape of the fault under investigation on the Fairphone
	 * 3: taps vanish with the panel otherwise healthy, and the only
	 * evidence is an interrupt that produced no frame. Say so out loud,
	 * rate limited, with the bytes, so the next occurrence answers whether
	 * a read really is coming back as zeroes. This only reports; the
	 * event is still processed exactly as before, so the measurement is
	 * not confounded by a change in behaviour.
	 */
	if (!memchr_inv(&event, 0, sizeof(event)))
		dev_warn_ratelimited(&ts->client->dev,
				     "all-zero event accepted by the checksum\n");
	else if (!himax_event_get_num_points(&event))
		dev_dbg(&ts->client->dev, "empty event: %*ph\n",
			(int)sizeof(event), &event);

	himax_process_event(ts, &event);

	return 0;
}

/*
 * Consecutive failed event reads before the controller is reset, and before the
 * handler starts throttling itself because the reset did not help either.
 */
#define HIMAX_ERRORS_BEFORE_RESET	10
#define HIMAX_ERRORS_BEFORE_BACKOFF	30
#define HIMAX_BACKOFF_MS		20

static irqreturn_t himax_irq_handler(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;
	int error;

	error = himax_handle_input(ts);
	if (!error) {
		ts->read_errors = 0;
		if (READ_ONCE(ts->fw_config_dirty))
			himax_apply_fw_config(ts);
		return IRQ_HANDLED;
	}

	/*
	 * The interrupt was ours: the controller raised it and this handler
	 * tried to service it. Returning IRQ_NONE says the opposite, and the
	 * kernel acts on that. The line is level triggered, so a controller
	 * that answers every read with an error re-asserts immediately and each
	 * pass is counted as unhandled - measured on an SDM632 board on
	 * 2026-09-05 at about 7800 passes a second, reaching the 100 000
	 * threshold in eleven seconds, at which point note_interrupt() disabled
	 * the touchscreen's interrupt outright and left a dead panel until the
	 * driver was rebound.
	 */
	ts->read_errors++;

	if (ts->read_errors == HIMAX_ERRORS_BEFORE_RESET) {
		dev_warn(&ts->client->dev,
			 "%u consecutive failed reads, resetting the controller\n",
			 ts->read_errors);
		himax_reset(ts);
		/* Reapplied on the next pass that reaches the part. */
		WRITE_ONCE(ts->fw_config_dirty, true);
	}

	/*
	 * If the reset did not help, stop spinning on the failure. This handler
	 * is threaded and may sleep; without the delay the failing path runs
	 * thousands of times a second, which costs power and buys nothing,
	 * because whatever the controller is waiting for is not this driver.
	 */
	if (ts->read_errors > HIMAX_ERRORS_BEFORE_BACKOFF)
		msleep(HIMAX_BACKOFF_MS);

	return IRQ_HANDLED;
}

/*
 * These are TDDI parts: the display half and the touch half are one die, and
 * the panel driver toggles the die's reset line every time it prepares the
 * panel. Measured on a Fairphone 3 across one screen-off/on cycle: the idle
 * mode switch this driver had cleared was back to the firmware default
 * afterwards, with no interrupt raised in between for the driver to notice. So
 * when the DT ties the touchscreen to its panel, follow it: hold the interrupt
 * off while the panel is down, and put the configuration back once the panel
 * driver has finished bringing the die up, which is well past the firmware's
 * reload window.
 */
static int himax_panel_prepared(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data,
						panel_follower);

	himax_apply_fw_config(ts);
	/* And have the first interrupt read it back once more. */
	WRITE_ONCE(ts->fw_config_dirty, true);
	enable_irq(ts->client->irq);
	return 0;
}

static int himax_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data,
						panel_follower);

	disable_irq(ts->client->irq);
	return 0;
}

static const struct drm_panel_follower_funcs himax_panel_follower_funcs = {
	.panel_prepared = himax_panel_prepared,
	.panel_unpreparing = himax_panel_unpreparing,
};

static int himax_probe(struct i2c_client *client)
{
	int error;
	struct device *dev = &client->dev;
	struct himax_ts_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->chip = i2c_get_match_data(client);

	ts->regmap = devm_regmap_init_i2c(client, &himax_regmap_config);
	error = PTR_ERR_OR_ZERO(ts->regmap);
	if (error) {
		dev_err(dev, "Failed to initialize regmap: %d\n", error);
		return error;
	}

	/*
	 * These parts are TDDI controllers: one die drives the display and the
	 * touch panel, and both halves live on the panel's rails. The display
	 * half is a separate DT node with its own iovcc-supply, so without a
	 * reference of our own the panel driver's vote is the only one - and
	 * when the display is powered down the touch controller loses its I/O
	 * supply while this driver still believes the device is reachable.
	 *
	 * Measured on a Fairphone 3 (hx83112b on BLSP1 QUP3): with the display
	 * off, the panel drops the rail, the controller stops driving the bus,
	 * and the next transfer holds both lines low until the QUP transfer
	 * timeout expires - 15 s on that board - after which the driver reports
	 * -ETIMEDOUT and drops the touch. Holding the rail here removes it.
	 *
	 * Boards that describe no supply get the dummy regulator, as before.
	 */
	error = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(himax_supplies),
					       himax_supplies);
	if (error)
		return dev_err_probe(dev, error, "Failed to enable supplies\n");

	/*
	 * When this driver's reference is what brought the rails up, the part
	 * is still powering on; a reset and a read issued immediately are
	 * NACKed and probe fails, releasing the rails again.
	 */
	msleep(HIMAX_POWER_ON_DELAY_MS);

	ts->gpiod_rst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	error = PTR_ERR_OR_ZERO(ts->gpiod_rst);
	if (error) {
		dev_err(dev, "Failed to get reset GPIO: %d\n", error);
		return error;
	}

	himax_reset(ts);

	if (ts->chip->check_id) {
		error = himax_check_product_id(ts);
		if (error)
			return error;
	}

	/*
	 * Let the firmware finish its reload, apply the configuration, and
	 * have the first interrupt read it back once more: a write that lands
	 * inside the reload window is silently undone.
	 */
	msleep(HIMAX_FW_RELOAD_MS);
	himax_apply_fw_config(ts);
	WRITE_ONCE(ts->fw_config_dirty, true);

	error = himax_input_register(ts);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					  himax_irq_handler, IRQF_ONESHOT,
					  client->name, ts);
	if (error)
		return error;

	if (IS_ENABLED(CONFIG_POWER_SUPPLY)) {
		ts->psy_nb.notifier_call = himax_psy_notifier;
		error = power_supply_reg_notifier(&ts->psy_nb);
		if (error)
			return error;

		error = devm_add_action_or_reset(dev, himax_psy_unreg_notifier,
						 &ts->psy_nb);
		if (error)
			return error;
	}

	if (drm_is_panel_follower(dev)) {
		/*
		 * The panel owns the interrupt from here: it is enabled by
		 * panel_prepared, which the follower core calls at once if the
		 * panel is already up.
		 */
		disable_irq(client->irq);
		ts->is_panel_follower = true;
		ts->panel_follower.funcs = &himax_panel_follower_funcs;
		error = devm_drm_panel_add_follower(dev, &ts->panel_follower);
		if (error)
			return dev_err_probe(dev, error,
					     "Failed to follow the panel\n");
	}

	return 0;
}

static int himax_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	/* The panel's unprepare already did this. */
	if (ts->is_panel_follower)
		return 0;

	disable_irq(ts->client->irq);
	return 0;
}

static int himax_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if (ts->is_panel_follower)
		return 0;

	enable_irq(ts->client->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(himax_pm_ops, himax_suspend, himax_resume);

static const struct himax_chip hx83100a_chip = {
	.read_events = hx83100a_read_events,
};

static const struct himax_chip hx83112b_chip = {
	.id = 0x83112b,
	.check_id = himax_check_product_id,
	.read_events = himax_read_events,
};

static const struct i2c_device_id himax_ts_id[] = {
	{ "hx83100a", (kernel_ulong_t)&hx83100a_chip },
	{ "hx83112b", (kernel_ulong_t)&hx83112b_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, himax_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id himax_of_match[] = {
	{ .compatible = "himax,hx83100a", .data = &hx83100a_chip },
	{ .compatible = "himax,hx83112b", .data = &hx83112b_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, himax_of_match);
#endif

static struct i2c_driver himax_ts_driver = {
	.probe = himax_probe,
	.id_table = himax_ts_id,
	.driver = {
		.name = "Himax-hx83112b-TS",
		.of_match_table = of_match_ptr(himax_of_match),
		.pm = pm_sleep_ptr(&himax_pm_ops),
	},
};
module_i2c_driver(himax_ts_driver);

MODULE_AUTHOR("Job Noorman <job@noorman.info>");
MODULE_DESCRIPTION("Himax hx83112b touchscreen driver");
MODULE_LICENSE("GPL");
