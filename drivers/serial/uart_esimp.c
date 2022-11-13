/*
*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT esimp_uart

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

/**
 * @brief Driver for UART port on ESIMP
 */


/* device config */
struct uart_esimp_config {
	uint8_t *base;
	int  parity;
	uart_irq_config_func_t irq_config_func;
};


/* driver data */
struct uart_esimp_data {
	/* Baud rate */
	uint32_t baud_rate;
	/* clock device */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};




int uart_esimp_init(const struct device *dev)
{
	const struct uart_esimp_config *config = (const struct uart_esimp_config *const)dev->config;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif
	return 0;
}


extern int uart_esimp_configure(const struct device *dev, const struct uart_config *cfg);
extern int uart_esimp_poll_in(const struct device *dev, unsigned char *c);
extern void uart_esimp_poll_out(const struct device *dev, unsigned char c);
extern int uart_esimp_err_check(const struct device *dev);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

void uart_esimp_isr(const struct device *dev)
{
	struct uart_esimp_data *data = (struct uart_esimp_data *const)dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}


void uart_esimp_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data)
{
	struct uart_esimp_data *data = (struct uart_esimp_data *const)dev->data;
	data->user_cb = cb;
	data->user_data = user_data;
}



extern int uart_esimp_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len);
extern int uart_esimp_fifo_read(const struct device *dev, uint8_t *rx_data, const int size);
extern void uart_esimp_irq_tx_enable(const struct device *dev);
extern void uart_esimp_irq_tx_disable(const struct device *dev);
extern int uart_esimp_irq_tx_ready(const struct device *dev);
extern void uart_esimp_irq_rx_enable(const struct device *dev);
extern void uart_esimp_irq_rx_disable(const struct device *dev);
extern int uart_esimp_irq_tx_complete(const struct device *dev);
extern int uart_esimp_irq_rx_ready(const struct device *dev);
extern void uart_esimp_irq_err_enable(const struct device *dev);
extern void uart_esimp_irq_err_disable(const struct device *dev);
extern int uart_esimp_irq_is_pending(const struct device *dev);
extern int uart_esimp_irq_update(const struct device *dev);

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_esimp_driver_api = {
	.configure = uart_esimp_configure,
	.config_get = NULL,
	.poll_in = uart_esimp_poll_in,
	.poll_out = uart_esimp_poll_out,
	.err_check = uart_esimp_err_check,
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	.fifo_fill = uart_esimp_fifo_fill,
	.fifo_read = uart_esimp_fifo_read,
	.irq_tx_enable = uart_esimp_irq_tx_enable,
	.irq_tx_disable = uart_esimp_irq_tx_disable,
	.irq_tx_ready = uart_esimp_irq_tx_ready,
	.irq_rx_enable = uart_esimp_irq_rx_enable,
	.irq_rx_disable = uart_esimp_irq_rx_disable,
	.irq_tx_complete = uart_esimp_irq_tx_complete,
	.irq_rx_ready = uart_esimp_irq_rx_ready,
	.irq_err_enable = uart_esimp_irq_err_enable,
	.irq_err_disable = uart_esimp_irq_err_disable,
	.irq_is_pending = uart_esimp_irq_is_pending,
	.irq_update = uart_esimp_irq_update,
	.irq_callback_set = uart_esimp_irq_callback_set,
#endif
};


#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
#define ESIMP_UART_IRQ_HANDLER_DECL(index)				\
	static void uart_esimp_irq_config_func_##index(const struct device *dev)
#define ESIMP_UART_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = uart_esimp_irq_config_func_##index,
#define ESIMP_UART_IRQ_HANDLER(index)					\
static void uart_esimp_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		DT_INST_IRQ(index, priority),				\
		uart_esimp_isr, DEVICE_DT_INST_GET(index),		\
		0);							\
	irq_enable(DT_INST_IRQN(index));				\
}
#else
#define ESIMP_UART_IRQ_HANDLER_DECL(index)
#define ESIMP_UART_IRQ_HANDLER_FUNC(index)
#define ESIMP_UART_IRQ_HANDLER(index)
#endif


#define ESIMP_UART_INIT(index)						\
ESIMP_UART_IRQ_HANDLER_DECL(index);					\
									\
static const struct uart_esimp_config uart_esimp_cfg_##index = {	\
	.base = (uint8_t *)DT_INST_REG_ADDR(index),			\
	.parity = DT_ENUM_IDX_OR(DT_DRV_INST(index), parity, UART_CFG_PARITY_NONE),	\
	ESIMP_UART_IRQ_HANDLER_FUNC(index)				\
};									\
									\
static struct uart_esimp_data uart_esimp_data_##index = {		\
	.baud_rate = DT_INST_PROP(index, current_speed),		\
};									\
									\
DEVICE_DT_INST_DEFINE(index,						\
		    &uart_esimp_init,					\
		    NULL,				\
		    &uart_esimp_data_##index, &uart_esimp_cfg_##index,	\
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
		    &uart_esimp_driver_api);				\
									\
ESIMP_UART_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(ESIMP_UART_INIT)
