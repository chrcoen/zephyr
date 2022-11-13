/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

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

void main(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
	struct uart_config cfg;
	cfg.baudrate = 115200;
	cfg.parity = UART_CFG_PARITY_EVEN;
	cfg.stop_bits = 1;
	cfg.data_bits = 8;
	cfg.flow_ctrl = 0;
	uart_configure(uart, &cfg);


	struct gpio_dt_spec tx_en;
	tx_en.port = DEVICE_DT_GET(DT_NODELABEL(gpioa));
	tx_en.pin = 0;
	tx_en.dt_flags = 0;
	if (tx_en.port == NULL) {
		printk("Could not find gpio device\n");
		return;
	}
	gpio_pin_configure_dt(&tx_en, GPIO_OUTPUT);

	printk("MCU1\n");
	uint8_t d = 0;
	for (;;) {
		uart_poll_in(uart, &d);
		printk("MCU1 receive %u\n", d);
		d++;
		printk("MCU1 send %u\n", d);
		k_busy_wait(500);
		gpio_pin_set_dt(&tx_en, 1);
		k_busy_wait(50);
		uart_poll_out(uart, d);
		k_busy_wait(500);
		gpio_pin_set_dt(&tx_en, 0);
		uart_poll_in(uart, &d);
	}
}


