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

K_SEM_DEFINE(my_sem, 0, 100);

const char *uptime()
{
	static char txt[16];
	uint32_t t = k_uptime_get_32();
	uint32_t t_sec = t / 1000;
	uint32_t t_ms = t % 1000;
	sprintf(txt, "%3d.%03d", t_sec, t_ms);
	return txt;
}

void uart_isr(const struct device *dev, void *user_data)
{
	uart_irq_update(dev);
	if (uart_irq_rx_ready(dev)) {
		uint8_t buffer[16];
		int len = 0;
		do {
			len = uart_fifo_read(dev, buffer, 16);
			for (int i = 0; i < len; i++) {
				printk("MCU0 receive %d %u\n", i, buffer[i]);
				k_sem_give(&my_sem);
			}
		} while (len >= 16);
	}
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

	k_msleep(10);

	uart_irq_callback_user_data_t cb;
	uart_irq_callback_set(uart, uart_isr);
	uart_irq_rx_enable(uart);


	uint8_t d = 5;
	for (;;) {
		printk("MCU0 send %u\n", d);
		gpio_pin_set_dt(&tx_en, 1);
		k_busy_wait(50);
		uart_poll_out(uart, d);
		k_busy_wait(500);
		gpio_pin_set_dt(&tx_en, 0);
		d++;
		while(k_sem_take(&my_sem, K_MSEC(50)) != 0) {
			printk("Input data not available!");
		}
		while(k_sem_take(&my_sem, K_MSEC(50)) != 0) {
			printk("Input data not available!");
		}
		k_busy_wait(500);
	}
}


