/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
 * Copyright (C) 2017 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/fb.h>
#include <linux/sched.h>

#include "gf_spi.h"

#include <linux/platform_device.h>

#define GF_DEV_NAME		"goodix_fp"
#define GF_IRQ_NAME		"goodix_fp-irq"
#define GF_DRIVER_NAME		"goodix_fp_spi"
#define GF_INPUT_NAME		"uinput-goodix"
#define GF_SPIDEV_NAME		"goodix,fingerprint"

static struct gf_device gf;

static void gf_hw_reset(struct gf_device *gf_dev, unsigned int delay_ms)
{
	gpio_set_value(gf_dev->reset_gpio, 0);
	msleep(delay_ms);
	gpio_set_value(gf_dev->reset_gpio, 1);
	msleep(delay_ms);
}

static void gf_irq_config(struct gf_device *gf_dev, bool state) {
	if (gf_dev->irq_enabled == state)
		return;

	if (state)
		enable_irq(gf_dev->irq);
	else
		disable_irq(gf_dev->irq);

	gf_dev->irq_enabled = state;
}

static void gf_kernel_key_input(struct gf_device *gf_dev, struct gf_key *gf_key)
{
	pr_debug("%s: received key, key=%d, value=%d\n",
			__func__, gf_key->key, gf_key->value);

	switch (gf_key->key) {
	case GF_KEY_HOME:
		input_report_key(gf_dev->input, GF_KEY_INPUT_HOME, gf_key->value);
		input_sync(gf_dev->input);
		break;
	}
}

extern bool capacitive_keys_enabled;

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_device *gf_dev = &gf;
	struct gf_key gf_key;
	int rc = 0;
	u8 netlink_route = NETLINK_TEST;

	switch (cmd) {
	case GF_IOC_INIT:
		pr_debug("%s: GF_IOC_INIT\n", __func__);
		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			rc = -EFAULT;
			break;
		}
		break;
	case GF_IOC_RESET:
		pr_debug("%s: GF_IOC_RESET.\n", __func__);
		gf_hw_reset(gf_dev, 3);
		break;
	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			pr_err("%s: failed to copy input key event\n", __func__);
			rc = -EFAULT;
			break;
		}
		if (capacitive_keys_enabled)
			gf_kernel_key_input(gf_dev, &gf_key);
		break;
	default:
		pr_debug("%s: unsupport cmd:0x%x\n", __func__, cmd);
	}

	return rc;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_device *gf_dev = handle;

	gf_dev->event = GF_NET_EVENT_IRQ;
	queue_work(gf_dev->event_workqueue, &gf_dev->event_work);

	return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = &gf;
	int rc;

	gf_dev->process = current;

	/*
	 * If this is not the first user, skip hardware configuration.
	 */
	if (++gf_dev->users != 1)
		goto no_config;

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("%s: failed to request reset_gpio, rc = %d\n", __func__, rc);
		goto error_reset_gpio;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("%s: failed to request irq_gpio, rc = %d\n", __func__, rc);
		goto error_irq_gpio;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	/*
	 * Requesting an irq also enables it.
	 */
	gf_dev->irq_enabled = true;
	rc = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			GF_IRQ_NAME, gf_dev);
	if (rc) {
		pr_err("%s: failed to request threaded irq, rc = %d\n", __func__, rc);
		goto error_irq_req;
	}

	gf_hw_reset(gf_dev, 3);

no_config:
	filp->private_data = gf_dev;
	nonseekable_open(inode, filp);

	return 0;

error_irq_req:
	gpio_free(gf_dev->irq_gpio);
error_irq_gpio:
	gpio_free(gf_dev->reset_gpio);
error_reset_gpio:
	return rc;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = filp->private_data;

	if (--gf_dev->users != 0)
		goto no_config;

	gf_irq_config(gf_dev, false);
	free_irq(gf_dev->irq, gf_dev);
	gpio_free(gf_dev->irq_gpio);
	gpio_free(gf_dev->reset_gpio);

no_config:
	filp->private_data = NULL;

	return 0;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
};

#define FINGERPRINT_PROCESSING_MS		2000
static void gf_event_worker(struct work_struct *work)
{
	struct gf_device *gf_dev = container_of(work, typeof(*gf_dev), event_work);
	char temp[4] = {0x0};

	/*
	 * If no process has opened the char device then no one is
	 * listening for the netlink event.
	 */
	if (!gf_dev->process)
		return;

	switch (gf_dev->event) {
	/*
	 * Elevate the fingerprint process priority when screen is off to ensure
	 * the fingerprint sensor is responsive and that the haptic
	 * response on successful verification always fires.
	 */
	case GF_NET_EVENT_FB_BLACK:
		gf_dev->display_on = false;
		set_user_nice(gf_dev->process, MIN_NICE);
		break;
	case GF_NET_EVENT_FB_UNBLACK:
		gf_dev->display_on = true;
		set_user_nice(gf_dev->process, 0);
		break;
	/*
	 * IRQs are followed by fingerprint procesing, hold a wakelock to make
	 * sure the fingerprint is processed when screen is off.
	 */
	case GF_NET_EVENT_IRQ:
		if (gf_dev->display_on)
			break;

		wake_lock_timeout(&gf_dev->fp_wakelock,
				msecs_to_jiffies(FINGERPRINT_PROCESSING_MS));
		break;
	}

	temp[0] = gf_dev->event;

	sendnlmsg(temp);
}

static int gf_fb_state_callback(struct notifier_block *nb,
		unsigned long type, void *data)
{
	struct fb_event *evdata = data;
	struct gf_device *gf_dev;
	unsigned int blank;

	if (type != FB_EVENT_BLANK)
		goto end;

	if (!evdata || !evdata->data)
		goto end;

	pr_debug("%s: type=%d\n", __func__, (int)type);

	gf_dev = container_of(nb, struct gf_device, notifier);

	blank = *(int *)(evdata->data);
	switch (blank) {
	case FB_BLANK_POWERDOWN:
		gf_dev->event = GF_NET_EVENT_FB_BLACK;
		break;
	case FB_BLANK_UNBLANK:
		gf_dev->event = GF_NET_EVENT_FB_UNBLACK;
		break;
	default:
		goto end;
	}

	queue_work(gf_dev->event_workqueue, &gf_dev->event_work);

end:
	return NOTIFY_OK;
}

static struct notifier_block gf_fb_notifier = {
	.notifier_call = gf_fb_state_callback,
};

static int gf_probe(struct platform_device *pdev)
{
	struct gf_device *gf_dev = &gf;
	struct device *dev;
	int major;
	int rc = 0;

	gf_dev->process = NULL;
	gf_dev->display_on = true;
	gf_dev->irq_enabled = false;

	gf_dev->reset_gpio = of_get_named_gpio(pdev->dev.of_node,
			"fp-gpio-reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_err("%s: failed to get reset_gpio, rc = %d\n", __func__, rc);
		rc = -EINVAL;
		goto error_dt;
	}

	gf_dev->irq_gpio = of_get_named_gpio(pdev->dev.of_node,
			"fp-gpio-irq", 0);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_err("%s: failed to get irq_gpio, rc = %d\n", __func__, rc);
		rc = -EINVAL;
		goto error_dt;
	}

	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);
	enable_irq_wake(gf_dev->irq);

	major = register_chrdev(0, GF_DRIVER_NAME, &gf_fops);
	if (major < 0) {
		pr_err("%s: failed to register char device\n", __func__);
		rc = major;
		goto error_chardev;
	}

	gf_dev->cls = class_create(THIS_MODULE, GF_DEV_NAME);
	if (IS_ERR(gf_dev->cls)) {
		pr_err("%s: failed to create device class\n", __func__);
		rc = PTR_ERR(gf_dev->cls);
		goto error_class;
	}

	gf_dev->devt = MKDEV(major, 0);
	dev = device_create(gf_dev->cls, &pdev->dev, gf_dev->devt,
				gf_dev, GF_DEV_NAME);
	if (IS_ERR(dev)) {
		pr_err("%s: failed to create device\n", __func__);
		rc = PTR_ERR(dev);
		goto error_device;
	}

	gf_dev->input = input_allocate_device();
	if (!gf_dev->input) {
		pr_err("%s: failed to allocate input device\n", __func__);
		rc = -ENOMEM;
		goto error_input_alloc;
	}

	input_set_capability(gf_dev->input, EV_KEY, GF_KEY_INPUT_HOME);

	gf_dev->input->name = GF_INPUT_NAME;
	rc = input_register_device(gf_dev->input);
	if (rc) {
		pr_err("%s: failed to register input device\n", __func__);
		goto error_input_register;
	}

	gf_dev->notifier = gf_fb_notifier;
	fb_register_client(&gf_dev->notifier);
	gf_dev->event_workqueue = alloc_workqueue("gf-event-wq",
			WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	INIT_WORK(&gf_dev->event_work, gf_event_worker);

	wake_lock_init(&gf_dev->fp_wakelock, WAKE_LOCK_SUSPEND, "fp_wakelock");

	netlink_init();

	return 0;

error_input_register:
	input_free_device(gf_dev->input);
error_input_alloc:
	device_destroy(gf_dev->cls, gf_dev->devt);
error_device:
	class_destroy(gf_dev->cls);
error_class:
	unregister_chrdev(MAJOR(gf_dev->devt), GF_DEV_NAME);
error_chardev:
error_dt:
	return rc;
}

static int gf_remove(struct platform_device *pdev)
{
	struct gf_device *gf_dev = &gf;

	netlink_exit();

	wake_lock_destroy(&gf_dev->fp_wakelock);

	destroy_workqueue(gf_dev->event_workqueue);

	fb_unregister_client(&gf_dev->notifier);

	input_unregister_device(gf_dev->input);

	device_destroy(gf_dev->cls, gf_dev->devt);
	class_destroy(gf_dev->cls);
	unregister_chrdev(MAJOR(gf_dev->devt), GF_DEV_NAME);

	return 0;
}

static struct of_device_id gf_match_table[] = {
	{ .compatible = GF_SPIDEV_NAME },
	{},
};

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gf_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

module_platform_driver(gf_driver);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
