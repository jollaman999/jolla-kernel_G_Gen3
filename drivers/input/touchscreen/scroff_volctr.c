/*
 * drivers/input/touchscreen/scroff_volctr.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2015, jollaman999 <admin@jollaman999.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input/scroff_volctr.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>

/* uncomment since no touchscreen defines android touch, do that here */
//#define ANDROID_TOUCH_DECLARED

/* if Sweep2Wake is compiled it will already have taken care of this */
#ifdef CONFIG_TOUCHSCREEN_SWEEP2WAKE
#define ANDROID_TOUCH_DECLARED
#endif

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "jollaman999 <admin@jollaman999.com>"
#define DRIVER_DESCRIPTION "Screen Off Volume Control for almost any device"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[scroff_volctr]: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

/* Tuneables */
#define SOVC_DEBUG		0
#define SOVC_DEFAULT		1
#define SOVC_FEATHER		400
#define SOVC_TIME_GAP		250

/* Resources */
int sovc_switch = SOVC_DEFAULT;
static cputime64_t touch_time_pre = 0;
static int touch_y = 0;
static int prev_y = 0;
static bool is_new_touch = false;
static bool is_vol_up = false;
static bool is_touching = false;
static bool scr_suspended = false;
static struct input_dev * sovc_input_volupdown;
static DEFINE_MUTEX(keyworklock);
static struct workqueue_struct *sovc_input_wq;
static struct work_struct sovc_input_work;

/* Read cmdline for sovc */
static int __init read_sovc_cmdline(char *sovc)
{
	if (strcmp(sovc, "1") == 0) {
		pr_info("[cmdline_sovc]: scroff_volctr enabled. | sovc='%s'\n", sovc);
		sovc_switch = 1;
	} else if (strcmp(sovc, "0") == 0) {
		pr_info("[cmdline_sovc]: scroff_volctr disabled. | sovc='%s'\n", sovc);
		sovc_switch = 0;
	} else {
		pr_info("[cmdline_sovc]: No valid input found. Going with default: | sovc='%u'\n", sovc_switch);
	}
	return 1;
}
__setup("sovc=", read_sovc_cmdline);

/* Voluem Key work func */
static void scroff_volctr_volupdown(struct work_struct * scroff_volctr_volupdown_work) {
	if (!mutex_trylock(&keyworklock))
		return;

	if (is_vol_up) {
		input_event(sovc_input_volupdown, EV_KEY, KEY_VOLUMEUP, 1);
		input_event(sovc_input_volupdown, EV_SYN, 0, 0);
		input_event(sovc_input_volupdown, EV_KEY, KEY_VOLUMEUP, 0);
		input_event(sovc_input_volupdown, EV_SYN, 0, 0);
	} else {
		input_event(sovc_input_volupdown, EV_KEY, KEY_VOLUMEDOWN, 1);
		input_event(sovc_input_volupdown, EV_SYN, 0, 0);
		input_event(sovc_input_volupdown, EV_KEY, KEY_VOLUMEDOWN, 0);
		input_event(sovc_input_volupdown, EV_SYN, 0, 0);
	}
	mutex_unlock(&keyworklock);
	return;
}
static DECLARE_WORK(scroff_volctr_volupdown_work, scroff_volctr_volupdown);

/* Voluem Key trigger */
static void scroff_volctr_volupdown_trigger(void) {
	schedule_work(&scroff_volctr_volupdown_work);
	return;
}

/* reset on finger release */
static void scroff_volctr_reset(void) {
	is_touching = false;
	prev_y = 0;
	is_new_touch = false;
}

/* init a new touch */
static void new_touch(int y) {
	touch_time_pre = ktime_to_ms(ktime_get());
	prev_y = y;
	is_new_touch = true;
}

/* scroff_volctr main function */
static void detect_scroff_volctr(int y)
{
	if (!is_touching) {
		if (!is_new_touch) {
			new_touch(y);
			return;
		}

		if (ktime_to_ms(ktime_get()) - touch_time_pre < SOVC_TIME_GAP) {
			// Volume Up (down->up)
			if (prev_y - y > SOVC_FEATHER) {
#ifdef SOVC_DEBUG
				pr_info(LOGTAG"UP\n");
#endif
				is_touching = true;
				is_vol_up = true;
				scroff_volctr_volupdown_trigger();
			}
			// Volume Down (up->down)
			else if (y - prev_y > SOVC_FEATHER) {
#ifdef SOVC_DEBUG
				pr_info(LOGTAG"DOWN\n");
#endif
				is_touching = true;
				is_vol_up = false;
				scroff_volctr_volupdown_trigger();
			}
		}
	}
}

static void sovc_input_callback(struct work_struct *unused) {

	detect_scroff_volctr(touch_y);

	return;
}

static void sovc_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	if ((!scr_suspended) || (!sovc_switch))
		return;

	/* You can debug here with 'adb shell getevent -l' command. */
	switch(code) {
		case ABS_MT_SLOT:
			scroff_volctr_reset();
			break;

		case ABS_MT_TRACKING_ID:
			if (value == 0xffffffff)
				scroff_volctr_reset();
			break;

		case ABS_MT_POSITION_Y:
			touch_y = value;
			queue_work_on(0, sovc_input_wq, &sovc_input_work);
			break;

		default:
			break;
	}
}

static int input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "touch"))
		return 0;
	else
		return 1;
}

static int sovc_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "sovc";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void sovc_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id sovc_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler sovc_input_handler = {
	.event		= sovc_input_event,
	.connect	= sovc_input_connect,
	.disconnect	= sovc_input_disconnect,
	.name		= "sovc_inputreq",
	.id_table	= sovc_ids,
};

static void sovc_early_suspend(struct early_suspend *h) {
	scr_suspended = true;
}

static void sovc_late_resume(struct early_suspend *h) {
	scr_suspended = false;
}

static struct early_suspend sovc_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = sovc_early_suspend,
	.resume = sovc_late_resume,
};

/*
 * SYSFS stuff below here
 */
static ssize_t sovc_scroff_volctr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", sovc_switch);

	return count;
}

static ssize_t sovc_scroff_volctr_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if ((buf[0] == '0' || buf[0] == '1') && buf[1] == '\n')
		if (sovc_switch != buf[0] - '0')
			sovc_switch = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(scroff_volctr, (S_IWUSR|S_IRUGO),
	sovc_scroff_volctr_show, sovc_scroff_volctr_dump);

static ssize_t sovc_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%s\n", DRIVER_VERSION);

	return count;
}

static ssize_t sovc_version_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(scroff_volctr_version, (S_IWUSR|S_IRUGO),
	sovc_version_show, sovc_version_dump);

/*
 * INIT / EXIT stuff below here
 */
#ifdef ANDROID_TOUCH_DECLARED
extern struct kobject *android_touch_kobj;
#else
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);
#endif
static int __init scroff_volctr_init(void)
{
	int rc = 0;

	sovc_input_volupdown = input_allocate_device();
	if (!sovc_input_volupdown) {
		pr_err("Can't allocate volume buttons to scroff_volctr\n");
		goto err_alloc_dev;
	}

	input_set_capability(sovc_input_volupdown, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(sovc_input_volupdown, EV_KEY, KEY_VOLUMEDOWN);
	sovc_input_volupdown->name = "sovc_volupdown";
	sovc_input_volupdown->phys = "sovc_volupdown/input0";

	rc = input_register_device(sovc_input_volupdown);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	sovc_input_wq = create_workqueue("sovciwq");
	if (!sovc_input_wq) {
		pr_err("%s: Failed to create sovciwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&sovc_input_work, sovc_input_callback);
	rc = input_register_handler(&sovc_input_handler);
	if (rc)
		pr_err("%s: Failed to register sovc_input_handler\n", __func__);

	register_early_suspend(&sovc_early_suspend_handler);

#ifndef ANDROID_TOUCH_DECLARED
	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
	}
#endif
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_scroff_volctr.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for scroff_volctr\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_scroff_volctr_version.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for scroff_volctr_version\n", __func__);
	}

err_input_dev:
	input_free_device(sovc_input_volupdown);
err_alloc_dev:
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit scroff_volctr_exit(void)
{
#ifndef ANDROID_TOUCH_DECLARED
	kobject_del(android_touch_kobj);
#endif
	input_unregister_handler(&sovc_input_handler);
	destroy_workqueue(sovc_input_wq);
	input_unregister_device(sovc_input_volupdown);
	input_free_device(sovc_input_volupdown);
	return;
}

module_init(scroff_volctr_init);
module_exit(scroff_volctr_exit);
