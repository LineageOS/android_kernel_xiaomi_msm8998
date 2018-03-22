/*
 * driver definition for sensor driver
 *
 * Coypright (c) 2017 Goodix
 * Copyright (C) 2017 XiaoMi, Inc.
 */
#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>

#define GF_KEY_INPUT_HOME		KEY_HOME

typedef enum gf_key_event {
	GF_KEY_HOME = 1
} gf_key_event_t;

struct gf_key {
	enum gf_key_event key;
	uint32_t value;
};

#define GF_IOC_MAGIC			'g'
#define GF_IOC_INIT			_IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_RESET			_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ		_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ		_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_INPUT_KEY_EVENT		_IOW(GF_IOC_MAGIC, 9, struct gf_key)

enum {
	GF_NET_EVENT_IRQ = 1,
	GF_NET_EVENT_FB_BLACK,
	GF_NET_EVENT_FB_UNBLACK
};

#define NETLINK_TEST 25

struct gf_device {
	struct class *cls;
	dev_t devt;

	struct input_dev *input;

	struct notifier_block notifier;
	struct workqueue_struct *event_workqueue;
	struct work_struct event_work;
	struct wake_lock fp_wakelock;
	bool display_on;
	int event;

	bool irq_enabled;
	signed irq_gpio;
	signed reset_gpio;

	unsigned users;
	int irq;
};

void sendnlmsg(char *message);
void netlink_init(void);
void netlink_exit(void);

#endif /*__GF_SPI_H*/
