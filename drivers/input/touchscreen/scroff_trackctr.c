/*
 * drivers/input/touchscreen/scroff_trackctr.c
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
#include <linux/input/scroff_trackctr.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>

/* ******************* HOW TO WORK *******************
 *  If you sweep touchscreen right to left in
 * SOTC_TIME_GAP (ms) time, you can play next track.
 *
 *  Otherwise if you sweep touchscreen left to right,
 * you can play previous track.
 */

/* uncomment since no touchscreen defines android touch, do that here */
//#define ANDROID_TOUCH_DECLARED

/* if Sweep2Wake is compiled it will already have taken care of this */
#ifdef CONFIG_TOUCHSCREEN_SWEEP2WAKE
#define ANDROID_TOUCH_DECLARED
#endif

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "jollaman999 <admin@jollaman999.com>"
#define DRIVER_DESCRIPTION "Screen Off Track Control for almost any device"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[scroff_trackctr]: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

/* Tuneables */
#define SOTC_DEBUG		0
#define SOTC_DEFAULT		1
#define SOTC_FEATHER		500
#define SOTC_TIME_GAP		250

/* Resources */
int sotc_switch = SOTC_DEFAULT;
static cputime64_t touch_time_pre = 0;
static int touch_x = 0;
static int prev_x = 0;
static bool is_new_touch = false;
static bool is_track_next = false;
static bool is_touching = false;
static bool scr_suspended = false;
static struct input_dev * sotc_input_nextpre;
static DEFINE_MUTEX(keyworklock);
static struct workqueue_struct *sotc_input_wq;
static struct work_struct sotc_input_work;

/* Read cmdline for sotc */
static int __init read_sotc_cmdline(char *sotc)
{
	if (strcmp(sotc, "1") == 0) {
		pr_info("[cmdline_sotc]: scroff_trackctr enabled. | sotc='%s'\n", sotc);
		sotc_switch = 1;
	} else if (strcmp(sotc, "0") == 0) {
		pr_info("[cmdline_sotc]: scroff_trackctr disabled. | sotc='%s'\n", sotc);
		sotc_switch = 0;
	} else {
		pr_info("[cmdline_sotc]: No valid input found. Going with default: | sotc='%u'\n", sotc_switch);
	}
	return 1;
}
__setup("sotc=", read_sotc_cmdline);

/* Track Key work func */
static void scroff_trackctr_nextpre(struct work_struct *scroff_trackctr_nextpre_work)
{
	if (!mutex_trylock(&keyworklock))
		return;

	if (is_track_next) {
#ifdef SOTC_DEBUG
		pr_info(LOGTAG"NEXT\n");
#endif
		input_event(sotc_input_nextpre, EV_KEY, KEY_NEXTSONG, 1);
		input_event(sotc_input_nextpre, EV_SYN, 0, 0);
		input_event(sotc_input_nextpre, EV_KEY, KEY_NEXTSONG, 0);
		input_event(sotc_input_nextpre, EV_SYN, 0, 0);
	} else {
#ifdef SOTC_DEBUG
		pr_info(LOGTAG"PREVIOUS\n");
#endif
		input_event(sotc_input_nextpre, EV_KEY, KEY_PREVIOUSSONG, 1);
		input_event(sotc_input_nextpre, EV_SYN, 0, 0);
		input_event(sotc_input_nextpre, EV_KEY, KEY_PREVIOUSSONG, 0);
		input_event(sotc_input_nextpre, EV_SYN, 0, 0);
	}
	mutex_unlock(&keyworklock);
}
static DECLARE_WORK(scroff_trackctr_nextpre_work, scroff_trackctr_nextpre);

/* Track Key trigger */
static void scroff_trackctr_nextpre_trigger(void)
{
	schedule_work(&scroff_trackctr_nextpre_work);
}

/* reset on finger release */
static void scroff_trackctr_reset(void)
{
	is_touching = false;
	is_new_touch = false;
	prev_x = 0;
}

/* init a new touch */
static void new_touch(int x)
{
	touch_time_pre = ktime_to_ms(ktime_get());
	is_new_touch = true;
	prev_x = x;
}

/* exec track control */
static void exec_trackctr(bool track_next)
{
	is_touching = true;
	is_track_next = track_next;
	scroff_trackctr_nextpre_trigger();
}

/* scroff_trackctr main function */
static void detect_scroff_trackctr(int x)
{
	if (!is_touching) {
		if (!is_new_touch)
			new_touch(x);

		if (ktime_to_ms(ktime_get()) - touch_time_pre < SOTC_TIME_GAP) {
			if (prev_x - x > SOTC_FEATHER) // Track Next (right->left)
				exec_trackctr(true);
			else if (x - prev_x > SOTC_FEATHER) // Track Previous (left->right)
				exec_trackctr(false);
		}
	}
}

static void sotc_input_callback(struct work_struct *unused)
{
	detect_scroff_trackctr(touch_x);
}

static void sotc_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	if ((!scr_suspended) || (!sotc_switch))
		return;

	/* You can debug here with 'adb shell getevent -l' command. */
	switch(code) {
		case ABS_MT_SLOT:
			scroff_trackctr_reset();
			break;

		case ABS_MT_TRACKING_ID:
			if (value == 0xffffffff)
				scroff_trackctr_reset();
			break;

		case ABS_MT_POSITION_X:
			touch_x = value;
			queue_work_on(0, sotc_input_wq, &sotc_input_work);
			break;

		default:
			break;
	}
}

static int input_dev_filter(struct input_dev *dev)
{
	if (strstr(dev->name, "touch"))
		return 0;
	else
		return 1;
}

static int sotc_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "sotc";

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

static void sotc_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id sotc_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler sotc_input_handler = {
	.event		= sotc_input_event,
	.connect	= sotc_input_connect,
	.disconnect	= sotc_input_disconnect,
	.name		= "sotc_inputreq",
	.id_table	= sotc_ids,
};

static void sotc_early_suspend(struct early_suspend *h)
{
	scr_suspended = true;
}

static void sotc_late_resume(struct early_suspend *h)
{
	scr_suspended = false;
}

static struct early_suspend sotc_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = sotc_early_suspend,
	.resume = sotc_late_resume,
};

/*
 * SYSFS stuff below here
 */
static ssize_t sotc_scroff_trackctr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", sotc_switch);

	return count;
}

static ssize_t sotc_scroff_trackctr_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if ((buf[0] == '0' || buf[0] == '1') && buf[1] == '\n')
		if (sotc_switch != buf[0] - '0')
			sotc_switch = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(scroff_trackctr, (S_IWUSR|S_IRUGO),
	sotc_scroff_trackctr_show, sotc_scroff_trackctr_dump);

static ssize_t sotc_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%s\n", DRIVER_VERSION);

	return count;
}

static ssize_t sotc_version_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(scroff_trackctr_version, (S_IWUSR|S_IRUGO),
	sotc_version_show, sotc_version_dump);

/*
 * INIT / EXIT stuff below here
 */
#ifdef ANDROID_TOUCH_DECLARED
extern struct kobject *android_touch_kobj;
#else
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);
#endif
static int __init scroff_trackctr_init(void)
{
	int rc = 0;

	sotc_input_nextpre = input_allocate_device();
	if (!sotc_input_nextpre) {
		pr_err("Can't allocate track buttons to scroff_trackctr\n");
		goto err_alloc_dev;
	}

	input_set_capability(sotc_input_nextpre, EV_KEY, KEY_NEXTSONG);
	input_set_capability(sotc_input_nextpre, EV_KEY, KEY_PREVIOUSSONG);
	sotc_input_nextpre->name = "sotc_nextpre";
	sotc_input_nextpre->phys = "sotc_nextpre/input0";

	rc = input_register_device(sotc_input_nextpre);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	sotc_input_wq = create_workqueue("sotciwq");
	if (!sotc_input_wq) {
		pr_err("%s: Failed to create sotciwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&sotc_input_work, sotc_input_callback);

	rc = input_register_handler(&sotc_input_handler);
	if (rc)
		pr_err("%s: Failed to register sotc_input_handler\n", __func__);

	register_early_suspend(&sotc_early_suspend_handler);

#ifndef ANDROID_TOUCH_DECLARED
	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
	}
#endif
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_scroff_trackctr.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for scroff_trackctr\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_scroff_trackctr_version.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for scroff_trackctr_version\n", __func__);
	}

err_input_dev:
	input_free_device(sotc_input_nextpre);
err_alloc_dev:
	pr_info(LOGTAG"%s done\n", __func__);

	return 0;
}

static void __exit scroff_trackctr_exit(void)
{
#ifndef ANDROID_TOUCH_DECLARED
	kobject_del(android_touch_kobj);
#endif
	input_unregister_handler(&sotc_input_handler);
	destroy_workqueue(sotc_input_wq);
	input_unregister_device(sotc_input_nextpre);
	input_free_device(sotc_input_nextpre);
}

module_init(scroff_trackctr_init);
module_exit(scroff_trackctr_exit);
