/*
 * DT2W 2.0 Driver - based on DT2W by Dennis Rassmann
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2018  Lukas Berger <mail@lukasberger.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/device.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <uapi/linux/fb.h>
#include <asm-generic/cputime.h>

/*
 * global tunables
 */
// stores the current DT2W-state set by the user
static bool dt2w_enabled = false;

// indicates if the DT2W-driver should use own wakelocks to
// ensure the availability of DT2W
static bool dt2w_wakelocks = true;

// indicates if DT2W should create an always-locked wake-lock
// while screen is off (WARNING: this will battery drain like butter)
static bool dt2w_wakelocks_static = true;

// maximal interval between two taps in which DT2W recognizes those
// taps as double-taps
static int dt2w_tap_interval = 700;

// maximal difference of both taps in pixels below which DT2W 
// recognizes those taps as wake-worthy double taps
static int dt2w_tap_offset = 200;

// defines for how many milliseconds the simulated power-button
// should be pressed
static int dt2w_power_key_duration = 60;

// indicates if the driver should print debugging-messages
static bool dt2w_debugging = false;

/*
 * DT2W-defs
 */
struct dt2w_coords {
	unsigned int x;
	unsigned int y;
	s64 time;
};

static void dt2w_handle_wake(struct work_struct *);

/*
 * DT2W-data
 */
// general
static bool dt2w_was_just_enabled = false;

// power key emulation
static struct input_dev *dt2w_power_key_dev;
static DEFINE_MUTEX(dt2w_power_key_lock);
static DECLARE_WORK(dt2w_power_key_work, dt2w_handle_wake);

// touch taps input processing
static struct workqueue_struct *dt2w_workqueue;
static bool dt2w_touch_track;
static struct dt2w_coords dt2w_touch_first;
static struct dt2w_coords dt2w_touch_second;

// screen state
static bool dt2w_screen_off = false;

// wakelocks
static struct wake_lock dt2w_wakelock_suspend;
static struct wake_lock dt2w_wakelock_input;
static bool dt2w_static_wakelock_enabled;

#define dt2w_now()  (ktime_to_ms(ktime_get()))

#define dt2w_error(fmt, ...)  pr_err("dt2w: %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define dt2w_warn(fmt, ...)  pr_warn("dt2w: %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define dt2w_info(fmt, ...)  pr_info("dt2w: %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define dt2w_debug(fmt, ...)  if (dt2w_debugging) pr_info("dt2w: %s: " fmt "\n", __func__, ##__VA_ARGS__)

/*
 * DT2W Global Stuff
 */
bool dt2w_is_enabled() {
	return dt2w_enabled;
}

bool dt2w_just_enabled() {
	return dt2w_was_just_enabled;
}

void dt2w_set_just_enabled(bool value) {
	dt2w_was_just_enabled = value;
}

/*
 * Wakelock handling
 */
static void dt2w_wake_lock(void) {
	if (dt2w_static_wakelock_enabled) {
		return;
	}

	dt2w_info("enabling static wakelock"); 
	wake_lock(&dt2w_wakelock_suspend);
	dt2w_static_wakelock_enabled = true;
}

static void dt2w_wake_unlock(void) {
	if (!dt2w_static_wakelock_enabled) {
		return;
	}

	dt2w_info("disabling static wakelock"); 
	wake_unlock(&dt2w_wakelock_suspend);
	dt2w_static_wakelock_enabled = false;
}

static void dt2w_wake_lock_timeout(long timeout) {
	if (dt2w_static_wakelock_enabled) {
		return;
	}

	dt2w_debug("setting wakelock for %ld jiffies", timeout);
	wake_lock_timeout(&dt2w_wakelock_input, timeout);
}

/*
 * DT2W Handling
 */
#define dt2w_trigger_key_power(_state) \
	input_event(dt2w_power_key_dev, EV_KEY, KEY_POWER, _state); \
	input_event(dt2w_power_key_dev, EV_SYN, 0, 0)

static void dt2w_handle_wake(struct work_struct *work) {
	// if we are already in here, it doesn't make sense
	// to trigger a POWER-event again
	if (!mutex_trylock(&dt2w_power_key_lock)) {
		dt2w_error("already locked");
		return;
	}

	dt2w_info("simulating power-key");

	dt2w_trigger_key_power(1);
	msleep(dt2w_power_key_duration);
	dt2w_trigger_key_power(0);

	mutex_unlock(&dt2w_power_key_lock);
}

/*
 * Input Handling
 */
static void dt2w_coords_reset(struct dt2w_coords *coords) {
	coords->x = -1;
	coords->y = -1;
	coords->time = 0;
}

#define dt2w_coords_reset_all() \
	dt2w_debug("resetting dt2w_touch_first"); \
	dt2w_coords_reset(&dt2w_touch_first); \
	dt2w_debug("resetting dt2w_touch_second"); \
	dt2w_coords_reset(&dt2w_touch_second)

static bool dt2w_coords_valid(struct dt2w_coords *coords) {
	return (coords->x != -1 && coords->y != -1);
}
 
static int dt2w_input_connect(struct input_handler *handler,
                              struct input_dev *dev,
                              const struct input_device_id *id) 
{
	struct input_handle *handle;
	int result;

	if (!strstr(dev->name, "touchscreen")) {
		dt2w_debug("device \"%s\" not suited for DT2W", dev->name);
		return -ENODEV;
	}

	dt2w_debug("using input-device \"%s\"", dev->name); 

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle) {
		dt2w_error("failed to kzalloc() for input-handle");
		return -ENOMEM;
	}

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "dt2w_input";

	result = input_register_handle(handle);
	if (result) {
		dt2w_error("failed to register input-handle");
		goto error_register_handle;
	}

	result = input_open_device(handle);
	if (result) {
		dt2w_error("failed to open input-device");
		goto error_open_device;
	}

	return 0;

error_open_device:
	input_unregister_handle(handle);
error_register_handle:
	kfree(handle);
	return result;
}

static void dt2w_input_event(struct input_handle *handle,
                             unsigned int type,
                             unsigned int code,
                             int value)
{
	struct dt2w_coords *coords;

	// either if screen is off or the we are expected to reset the
	// handler, reset both coords and exit
	if (!dt2w_enabled || !dt2w_screen_off || code == ABS_MT_SLOT) {
		dt2w_coords_reset_all();
		dt2w_debug("not processing input-event %d (0x%X)", code, code);
		return;
	}

	// only trigger the wakelock if they are requested
	// AND we aren't already using the static one
	if (dt2w_wakelocks && !dt2w_wakelocks_static) {
		dt2w_wake_lock_timeout(HZ / 2);
	}

	// check if the event is a screen-was-tapped-event
	// and set the flag
	if (code == ABS_MT_TRACKING_ID) {
		dt2w_touch_track = true;
		dt2w_debug("registered screen-was-tapped");
		return;
	}

	// if this event wasn't preceeded by a screen-was-tapped-event,
	// skip it
	if (!dt2w_touch_track) {
		dt2w_debug("not a screen-tap event %d (0x%X), skipping", code, code);
		return;
	}

	// check if we even need to start to process the event
	if (code != ABS_MT_POSITION_X && code != ABS_MT_POSITION_Y) {
		dt2w_debug("unused event %d (0x%X), early-skipping...", code, code);
		return;
	}

	// if the first coords are valid, continue with filling
	// the second coords, else begin with the first ones
	if (dt2w_coords_valid(&dt2w_touch_first)) {
		coords = &dt2w_touch_second;
		dt2w_debug("selected <dt2w_touch_second>");
	} else {
		coords = &dt2w_touch_first;
		dt2w_debug("selected <dt2w_touch_first>");
	}

	if (code == ABS_MT_POSITION_X) {
		coords->x = value;
		dt2w_debug("coords->x = <%d>", coords->x);
	} else if (code == ABS_MT_POSITION_Y) {
		coords->y = value;
		dt2w_debug("coords->y = <%d>", coords->y);
	}

	// store time if both coordinates have been confirmed
	if (dt2w_coords_valid(coords)) {
		// update time
		coords->time = dt2w_now();

		// as we processed the touch just now, we have to reset the
		// screen-was-just-tapped-indicator
		dt2w_touch_track = false;

		dt2w_debug("touch at %dx%d", coords->x, coords->y);
	}

	if (dt2w_coords_valid(&dt2w_touch_first) && dt2w_coords_valid(&dt2w_touch_second)) {
		unsigned int dx = abs(dt2w_touch_second.x - dt2w_touch_first.x);
		unsigned int dy = abs(dt2w_touch_second.y - dt2w_touch_first.y);
		unsigned int dt = (unsigned int)(dt2w_touch_second.time - dt2w_touch_first.time);

		dt2w_debug("got two valid touch-taps, validating diffs");
		dt2w_debug("dx = <%d> (limit <%d>)", dx, dt2w_tap_offset);
		dt2w_debug("dy = <%d> (limit <%d>)", dy, dt2w_tap_offset);
		dt2w_debug("dt = <%d> (limit <%d>)", dt, dt2w_tap_interval);

		// check limitations
		//   1. check if x-coordinate is in between 0 and the maximal offset
		//   2. check if x-coordinate is in between 0 and the maximal offset
		//   3. check if the time-interval is in the set range
		if (dx < dt2w_tap_offset && dy < dt2w_tap_offset && dt < dt2w_tap_interval) {
			// we are ready to wake the device
			dt2w_debug("scheduling work for power-key simulation");
			schedule_work(&dt2w_power_key_work);
			
			// event handled, reset all coords
			dt2w_coords_reset_all();
			return;
		}

		// on a <press1> - <long pause> - <press2> - <short pause> - <press3>
		// situation DT2W would normally fail because it would drop press1 and
		// press2, leaving press3 waiting for press4 which may not come/delay
		// Thus, if the interval-ranges weren't met, move press2 from secondary
		// slot to first slot to make it usable for press3 and reset secondary slot
		dt2w_debug("moving 2nd slot to 1st slot for re-using");
		memcpy(&dt2w_touch_first, &dt2w_touch_second, sizeof(struct dt2w_coords));
		dt2w_coords_reset(&dt2w_touch_second);
	}
}

static void dt2w_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dt2w_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler dt2w_input_handler = {
	.event		= dt2w_input_event,
	.connect	= dt2w_input_connect,
	.disconnect	= dt2w_input_disconnect,
	.name		= "dt2w_input_handler",
	.id_table	= dt2w_ids,
};

/*
 * sysfs Confgiuration Stuff
 */
#define dt2w_sysfs_attr(_name) \
	__ATTR(_name, S_IWUSR | S_IRUGO, dt2w_sysfs_##_name##_show, dt2w_sysfs_##_name##_store)

#define dt2w_sysfs_show(_name) \
	static ssize_t dt2w_sysfs_##_name##_show(struct class *class, \
	                                         struct class_attribute *attr, \
	                                         char *buf) \
	{ \
		return scnprintf(buf, PAGE_SIZE, "%d\n", dt2w_##_name); \
	}

#define __dt2w_sysfs_store(_name, _conv, _handler) \
	static void (*dt2w_sysfs_##_name##_store_handler)(void) = _handler; \
	static ssize_t dt2w_sysfs_##_name##_store(struct class *class, \
	                                          struct class_attribute *attr, \
	                                          const char *buf, \
	                                          size_t count) \
	{ \
		unsigned long val = 0; \
		int ret = kstrtoul(buf, 0, &val); \
		if (ret < 0) \
			return ret; \
		dt2w_##_name = _conv(val); \
		if (dt2w_sysfs_##_name##_store_handler) \
			dt2w_sysfs_##_name##_store_handler(); \
		return count; \
	}

#define dt2w_sysfs_conv_int(data)  ((int)data)
#define dt2w_sysfs_store_int(_name, _handler) \
	__dt2w_sysfs_store(_name, dt2w_sysfs_conv_int, _handler);

#define dt2w_sysfs_conv_bool(data)  (!!(data))
#define dt2w_sysfs_store_bool(_name, _handler) \
	__dt2w_sysfs_store(_name, dt2w_sysfs_conv_bool, _handler);

static void dt2w_sysfs_wakelocks_handler(void) {
	if (!dt2w_wakelocks_static) {
		dt2w_wake_unlock();
	} else if (dt2w_enabled && dt2w_screen_off && dt2w_wakelocks && dt2w_wakelocks_static) {
		dt2w_wake_lock();
	}
}

dt2w_sysfs_show(enabled);
dt2w_sysfs_show(wakelocks);
dt2w_sysfs_show(wakelocks_static);
dt2w_sysfs_show(tap_interval);
dt2w_sysfs_show(tap_offset);
dt2w_sysfs_show(power_key_duration);
dt2w_sysfs_show(debugging);

dt2w_sysfs_store_bool(enabled, NULL);
dt2w_sysfs_store_bool(wakelocks, dt2w_sysfs_wakelocks_handler);
dt2w_sysfs_store_bool(wakelocks_static, dt2w_sysfs_wakelocks_handler);
dt2w_sysfs_store_int(tap_interval, NULL);
dt2w_sysfs_store_int(tap_offset, NULL);
dt2w_sysfs_store_int(power_key_duration, NULL);
dt2w_sysfs_store_bool(debugging, NULL);

static struct class_attribute dt2w_sysfs_class_attrs[] = {
    dt2w_sysfs_attr(enabled),
    dt2w_sysfs_attr(wakelocks),
    dt2w_sysfs_attr(wakelocks_static),
    dt2w_sysfs_attr(tap_interval),
    dt2w_sysfs_attr(tap_offset),
	dt2w_sysfs_attr(power_key_duration),
    dt2w_sysfs_attr(debugging),
    __ATTR_NULL,
};

static struct class dt2w_sysfs_class = {
    .name        = "dt2w",
    .class_attrs = dt2w_sysfs_class_attrs,
};

/*
 * Screen State Notifier
 */
static int dt2w_lcd_notifier_call(struct notifier_block *nb,
                                  unsigned long val,
                                  void *data)
{
	struct fb_event *evdata = (struct fb_event *)data;
	int fb_state = -1;

	dt2w_debug("enter");

	if (val != FB_EVENT_BLANK && val != FB_R_EARLY_EVENT_BLANK) {
		dt2w_debug("invalid value <%ld>", val);
		return 0;
	}

	/*
	 * If FBNODE is not zero, it is not primary display(LCD)
	 * and don't need to process these scheduling.
	 */
	if (evdata->info->node) {
		dt2w_debug("node not zero (not primary display)");
		return NOTIFY_OK;
	}

	fb_state = *(int *)evdata->data;
	switch (fb_state) {
		case FB_BLANK_POWERDOWN:
			dt2w_screen_off = true;
			dt2w_was_just_enabled = true;
			dt2w_info("dt2w_screen_off = %d", dt2w_screen_off);

			if (dt2w_enabled && dt2w_wakelocks && dt2w_wakelocks_static)
				dt2w_wake_lock();
			break;

		case FB_BLANK_UNBLANK:
			dt2w_screen_off = false;
			dt2w_was_just_enabled = false;
			dt2w_info("dt2w_screen_off = %d", dt2w_screen_off);

			dt2w_wake_unlock();
			break;

		default:
			dt2w_debug("invalid fb_state <%d>", fb_state);
			break;
	}

	return NOTIFY_OK;
}

static struct notifier_block dt2w_lcd_notifier_block = {
	.notifier_call = dt2w_lcd_notifier_call,
};

static int __init dt2w_init(void) {
	int result = 0;

	dt2w_power_key_dev = input_allocate_device();
	if (!dt2w_power_key_dev) {
		dt2w_error("failed to allocate power-key simulation input device");
		result = -ENOMEM;
		goto error_allocate_power_key_input_dev;
	}
	dt2w_debug("allocated power-key simulation input device!");

	input_set_capability(dt2w_power_key_dev, EV_KEY, KEY_POWER);
	dt2w_power_key_dev->name = "dt2w_power_key";
	dt2w_power_key_dev->phys = "dt2w_power_key/input0";

	result = input_register_device(dt2w_power_key_dev);
	if (result) {
		dt2w_error("failed to register power-key simulation input device (%d)", -result);
		goto error_register_power_key_input_dev;
	}
	dt2w_debug("registered power-key simulation input device!");

	result = fb_register_client(&dt2w_lcd_notifier_block);
	if (result) {
		dt2w_error("failed to register LCD state-change notifier (%d)", -result);
		goto error_register_lcd_notifier;
	}
	dt2w_debug("registered LCD state-change notifier!");

	dt2w_workqueue = create_workqueue("dt2w-workqueue");
	if (!dt2w_workqueue) {
		dt2w_error("failed to create DT2W-workqueue (%d)", -result);
		goto error_create_workqueue;
	}
	dt2w_debug("created DT2W-workqueue!");

	result = input_register_handler(&dt2w_input_handler);
	if (result) {
		dt2w_error("failed to register input-handler (%d)", -result);
		goto error_register_input_handler;
	}
	dt2w_debug("registered input-handler!");

	result = class_register(&dt2w_sysfs_class);
	if (result) {
		dt2w_error("failed to register sysfs-class (%d)", -result);
		goto error_class_register;
	}
	dt2w_debug("registered sysfs-class!");

	wake_lock_init(&dt2w_wakelock_input, WAKE_LOCK_SUSPEND, "dt2w-input");
	wake_lock_init(&dt2w_wakelock_suspend, WAKE_LOCK_SUSPEND, "dt2w-suspend");

	dt2w_coords_reset_all();

	dt2w_info("succeeded with 0");
	return 0;

error_class_register:
	input_unregister_handler(&dt2w_input_handler);

error_register_input_handler:
	destroy_workqueue(dt2w_workqueue);

error_create_workqueue:
	fb_unregister_client(&dt2w_lcd_notifier_block);

error_register_lcd_notifier:
	input_unregister_device(dt2w_power_key_dev);

error_register_power_key_input_dev:
	input_free_device(dt2w_power_key_dev);

error_allocate_power_key_input_dev:
	dt2w_info("failed with %d", -result);
	return result;
}

static void __exit dt2w_exit(void) {
	class_unregister(&dt2w_sysfs_class);

	input_unregister_handler(&dt2w_input_handler);

	destroy_workqueue(dt2w_workqueue);

	fb_unregister_client(&dt2w_lcd_notifier_block);

	input_unregister_device(dt2w_power_key_dev);
	input_free_device(dt2w_power_key_dev);

	wake_lock_destroy(&dt2w_wakelock_input);
	wake_lock_destroy(&dt2w_wakelock_suspend);
}

late_initcall(dt2w_init);
module_exit(dt2w_exit);

MODULE_AUTHOR("Lukas Berger <mail@lukasberger.at>");
MODULE_DESCRIPTION("generic doubletap2wake input driver");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPLv2");
