/*
 * netlink interface
 *
 * Copyright (c) 2017 Goodix
 * Copyright (C) 2017 XiaoMi, Inc.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>

#include "gf_spi.h"

#define MAX_MSG_SIZE 32

static int pid = -1;
struct sock *nl_sk = NULL;

void sendnlmsg(char *message)
{
	struct sk_buff *skb_1;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(MAX_MSG_SIZE);
	int ret = 0;
	int slen;

	if (!message || !nl_sk || !pid)
		return;

	skb_1 = alloc_skb(len, GFP_KERNEL);
	if (!skb_1) {
		pr_err("%s: failed to allocate network buffer\n", __func__);
		return;
	}

	nlh = nlmsg_put(skb_1, 0, 0, 0, MAX_MSG_SIZE, 0);

	NETLINK_CB(skb_1).portid = 0;
	NETLINK_CB(skb_1).dst_group = 0;

	slen = strlen(message);
	message[slen] = '\0';
	memcpy(NLMSG_DATA(nlh), message, slen + 1);

	ret = netlink_unicast(nl_sk, skb_1, pid, MSG_DONTWAIT);
	if (!ret)
		pr_err("%s: failed to send netlink message\n", __func__);
}


void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[100];

	skb = skb_get (__skb);

	if (skb->len < NLMSG_SPACE(0))
		return;

	nlh = nlmsg_hdr(skb);
	memcpy(str, NLMSG_DATA(nlh), sizeof(str));
	pid = nlh->nlmsg_pid;

	kfree_skb(skb);
}


void netlink_init(void)
{
	struct netlink_kernel_cfg netlink_cfg;

	memset(&netlink_cfg, 0, sizeof(struct netlink_kernel_cfg));
	netlink_cfg.groups = 0;
	netlink_cfg.flags = 0;
	netlink_cfg.input = nl_data_ready;
	netlink_cfg.cb_mutex = NULL;

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST,
			&netlink_cfg);
	if (!nl_sk)
		pr_err("%s: failed to create netlink socket\n", __func__);
}

void netlink_exit(void)
{
	if (!nl_sk)
		return;

	netlink_kernel_release(nl_sk);
	nl_sk = NULL;
}

