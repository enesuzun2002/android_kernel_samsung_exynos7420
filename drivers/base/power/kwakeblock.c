/*
 * Generic Kernel Wakelock-Blocker Driver
 *
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#define KWAKEBLOCK_MAX_NAME_LEN  255

struct kwakeblock_item {
    struct list_head list;
    char name[KWAKEBLOCK_MAX_NAME_LEN];
	size_t length;
	atomic_long_t count;
};

/*
 * Tunables
 */
// determines if kwakeblock is activate and should block
// wakeup-sources. default to <false>.
static bool kwakeblock_enabled = false;

/*
 * Data
 */
static struct list_head kwakeblock_list;
static atomic_t kwakeblock_initialized = ATOMIC_INIT(0);

/*
 * Global methods
 */
bool kwakeblock_is_blocked(const char *name) {
	struct kwakeblock_item *wakeblock, *n;

	// check if kwakeblock was initialized
	if (!atomic_read(&kwakeblock_initialized))
		return false;

	// check if wakeblock is enabled
	if (unlikely(!kwakeblock_enabled))
		return false;

	// go throught the list of blocked wakelocks
	list_for_each_entry_safe(wakeblock, n, &kwakeblock_list, list)
	{
		if (!memcmp(wakeblock->name, name, wakeblock->length)) {
			// if we create statistics, increase counter
			atomic_long_inc(&wakeblock->count);

			// we found the name in the block-list
			return true;
		}
	}

	// not found, so not blocked
	return false;
}

/*
 * sysfs Configuration Stuff
 */
static ssize_t kwakeblock_sysfs_enabled_show(struct class *class,
                                             struct class_attribute *attr,
                                             char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", kwakeblock_enabled);
}

static ssize_t kwakeblock_sysfs_enabled_store(struct class *class,
                                              struct class_attribute *attr,
                                              const char *buf,
                                              size_t count)
{
	unsigned long val = 0;
	int ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	kwakeblock_enabled = !!val;
	return count;
}

static ssize_t kwakeblock_sysfs_list_show(struct class *class,
                                          struct class_attribute *attr,
                                          char *buf)
{
	struct kwakeblock_item *wakeblock, *n;
	int count = 0;

	memset(buf, 0, PAGE_SIZE);

	list_for_each_entry_safe(wakeblock, n, &kwakeblock_list, list)
	{
		count = scnprintf(buf, PAGE_SIZE, "%s\"%s\"\t%ld\n", buf, wakeblock->name, atomic_long_read(&wakeblock->count));
	}

	return count;
}

static ssize_t kwakeblock_sysfs_reset_stats_store(struct class *class,
                                                  struct class_attribute *attr,
                                                  const char *buf,
                                                  size_t count)
{
	struct kwakeblock_item *wakeblock, *n;

	list_for_each_entry_safe(wakeblock, n, &kwakeblock_list, list)
	{
		pr_info("%s: resetting stats for kwakeblock \"%s\"", __func__, wakeblock->name);

		// resetting stats
		atomic_long_set(&wakeblock->count, 0);
	}

	return count;
}

static ssize_t kwakeblock_sysfs_block_store(struct class *class,
                                            struct class_attribute *attr,
                                            const char *buf,
                                            size_t count)
{
	struct kwakeblock_item *wakeblock, *n;
	const char *buf_string;
	size_t buf_string_len = count;

	// remove newline from input
	buf_string = memchr(buf, '\n', count);
	if (buf_string)
		buf_string_len = buf_string - buf;

	// check if the given name in between limits
	if (buf_string_len <= 0 || buf_string_len >= KWAKEBLOCK_MAX_NAME_LEN)
		return -EINVAL;

	// check if this wakeblock is already registered
	list_for_each_entry_safe(wakeblock, n, &kwakeblock_list, list)
	{
		if (!memcmp(wakeblock->name, buf, wakeblock->length - 1)) {
			// already blocked, exit
			pr_info("%s: kwakeblock \"%.*s\" already added", __func__, (int)buf_string_len, buf);
			return count;
		}
	}

	pr_info("%s: adding kwakeblock \"%.*s\"", __func__, (int)buf_string_len, buf);

	// allocate memory
	wakeblock = kzalloc(sizeof(struct kwakeblock_item), GFP_KERNEL);

	// set data
	memcpy(wakeblock->name, buf, buf_string_len);
	wakeblock->length = count;
	atomic_long_set(&wakeblock->count, 0);

	// add to wakeblock-list
	list_add_tail(&wakeblock->list, &kwakeblock_list);

	return count;
}

static ssize_t kwakeblock_sysfs_unblock_store(struct class *class,
                                              struct class_attribute *attr,
                                              const char *buf,
                                              size_t count)
{
	struct kwakeblock_item *wakeblock, *n;
	const char *buf_string;
	size_t buf_string_len = count;

	// remove newline from input
	buf_string = memchr(buf, '\n', count);
	if (buf_string)
		buf_string_len = buf_string - buf;

	// check if the given name in between limits
	if (buf_string_len <= 0 || buf_string_len >= KWAKEBLOCK_MAX_NAME_LEN)
		return -EINVAL;

	list_for_each_entry_safe(wakeblock, n, &kwakeblock_list, list)
	{
		if (!memcmp(wakeblock->name, buf, wakeblock->length - 1)) {
			pr_info("%s: removing kwakeblock \"%s\"", __func__, wakeblock->name);

			// remove from wakeblock-list
			list_del(&wakeblock->list);

			// clean up
			kfree(wakeblock);

			// early-exit
			return count;
		}
	}

	pr_info("%s: no kwakeblock with the name \"%.*s\" found", __func__, (int)buf_string_len, buf);
	return count;
}

static ssize_t kwakeblock_sysfs_unblock_all_store(struct class *class,
                                                  struct class_attribute *attr,
                                                  const char *buf,
                                                  size_t count)
{
	struct kwakeblock_item *wakeblock, *n;

	list_for_each_entry_safe(wakeblock, n, &kwakeblock_list, list)
	{
		pr_info("%s: removing kwakeblock \"%s\"", __func__, wakeblock->name);

		// remove from wakeblock-list
		list_del(&wakeblock->list);

		// clean up
		kfree(wakeblock);
	}

	return count;
}

static struct class_attribute kwakeblock_sysfs_class_attrs[] = {
    __ATTR(enabled, 0660, kwakeblock_sysfs_enabled_show, kwakeblock_sysfs_enabled_store),

    __ATTR(list, 0440, kwakeblock_sysfs_list_show, NULL),
    __ATTR(reset_stats, 0220, NULL, kwakeblock_sysfs_reset_stats_store),

    __ATTR(block, 0220, NULL, kwakeblock_sysfs_block_store),
    __ATTR(unblock, 0220, NULL, kwakeblock_sysfs_unblock_store),
    __ATTR(unblock_all, 0220, NULL, kwakeblock_sysfs_unblock_all_store),
    __ATTR_NULL,
};

static struct class kwakeblock_sysfs_class = {
    .name        = "kwakeblock",
    .class_attrs = kwakeblock_sysfs_class_attrs,
};

static int __init kwakeblock_init(void) {
	int result;

	// register userspace configuration interface
	result = class_register(&kwakeblock_sysfs_class);
	if (result) {
		pr_err("%s: failed to register sysfs-class", __func__);
		goto error_class_register;
	}

	// initialize wakelock block list
	INIT_LIST_HEAD(&kwakeblock_list);

	// signalize we are ready
	atomic_set(&kwakeblock_initialized, 1);

error_class_register:
	return result;
}

static void __exit kwakeblock_exit(void) {
	class_unregister(&kwakeblock_sysfs_class);
}

late_initcall(kwakeblock_init);
module_exit(kwakeblock_exit);

MODULE_AUTHOR("Lukas Berger <mail@lukasberger.at>");
MODULE_DESCRIPTION("Generic Kernel Wakelock-Blocker Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPLv2");
