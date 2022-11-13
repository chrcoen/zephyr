/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#define STACKSIZE 1024

#define PRIORITY 7



const char *uptime()
{
	static char txt[16];
	uint32_t t = k_uptime_get_32();
	uint32_t t_sec = t / 1000;
	uint32_t t_ms = t % 1000;
	sprintf(txt, "%3d.%03d", t_sec, t_ms);
	return txt;
}

void thread1(void)
{
	int count = 0;
	for (;;) {
		printk("%s - Thread 1: %d\n", uptime(), count);
		k_msleep(300);
		count++;
	}
}


void main(void)
{
	int count = 0;
	for (;;) {
		printk("%s - Main: %d\n", uptime(), count);
		k_msleep(1000);
		count++;
	}
}


K_THREAD_DEFINE(thread1_id, STACKSIZE, thread1, NULL, NULL, NULL,
		PRIORITY, 0, 0);

