/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT esimp_gpio

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>



/**
 * @brief configuration of GPIO device
 */
struct gpio_esimp_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* IO port */
	int port;
};

/**
 * @brief driver data
 */
struct gpio_esimp_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* device's owner of this data */
	const struct device *dev;
	/* user ISR cb */
	sys_slist_t cb;
};


extern int gpio_esimp_init(const struct device *dev);
extern int gpio_esimp_configure(const struct device *dev, gpio_pin_t pin,
			     gpio_flags_t flags);
extern int gpio_esimp_port_get_raw(const struct device *dev, uint32_t *value);
extern int gpio_esimp_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value);
extern int gpio_esimp_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins);
extern int gpio_esimp_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins);
extern int gpio_esimp_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins);
extern int gpio_esimp_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig);
extern int gpio_esimp_manage_callback(const struct device *dev,
				      struct gpio_callback *callback,
				      bool set);

static const struct gpio_driver_api gpio_esimp_driver = {
	.pin_configure = gpio_esimp_configure,
	.port_get_raw = gpio_esimp_port_get_raw,
	.port_set_masked_raw = gpio_esimp_port_set_masked_raw,
	.port_set_bits_raw = gpio_esimp_port_set_bits_raw,
	.port_clear_bits_raw = gpio_esimp_port_clear_bits_raw,
	.port_toggle_bits = gpio_esimp_port_toggle_bits,
	.pin_interrupt_configure = gpio_esimp_pin_interrupt_configure,
	.manage_callback = gpio_esimp_manage_callback,
};


static const struct gpio_esimp_config gpio_esimp_config = {
	.common = {
		.port_pin_mask = 0xffffffff,
	},
	.port = 0,
};

static struct gpio_esimp_data gpio_esimp_data = {
	.common = {
		.invert = 0,
	},
	.dev = NULL,
	.cb = NULL
};

DEVICE_DT_DEFINE(
	DT_NODELABEL(gpioa),
	gpio_esimp_init,
	NULL,
	&gpio_esimp_data,
	&gpio_esimp_config,
	PRE_KERNEL_1,
	CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	&gpio_esimp_driver
);
