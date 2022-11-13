#include "esimp/interface/platform_if.hpp"


#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <kernel_internal.h>
#include <ksched.h>
#include <kswap.h>

#include <cstdio>
#include <cstring>
#include <assert.h>
#include <map>
#include <cstring>

#if defined(CONFIG_TICKLESS_KERNEL)
#error "Tickeless not supported yet"
#endif


#define _EXC_IRQ_DEFAULT_PRIO 2


extern "C" void z_cstart(void);

extern "C" typedef void (* IsrFctPtr)(const void *);

struct ThreadInfo {
  esimp::Thread_if *sim_thread;
  k_thread_entry_t entry;
  void *p1;
  void *p2;
  void *p3;
  int aborted;
};

struct IsrInfo {
  IsrFctPtr fct;
  const void *param;
};

static esimp::MCU_if *mcu;
static esimp::Timer_if *systick;


class Application : public esimp::Application_if {
public:
  virtual void run_main(esimp::Thread_if *thread) override {
    // log_init();
    z_cstart();
  }

  virtual void run_thread(esimp::Thread_if *thread, const char *name, void *pdata) override {
    const ThreadInfo* t = static_cast<ThreadInfo *>(pdata);
     z_thread_entry(t->entry, t->p1, t->p2, t->p3);
    assert(0);
  }

  virtual void run_isr(esimp::IRQ_if *irq) override {
    IsrInfo *inf = (IsrInfo*)irq->get_pdata();
    assert(inf != nullptr);
    inf->fct(inf->param);
    if (_kernel.ready_q.cache != _current) {
      unsigned int irq_lock = 0;
      z_swap_irqlock(irq_lock);
    }
  }
};

static Application app;

extern "C" void systick_isr(const void *pdata)
{
  systick->clear_irq();
  sys_clock_announce(1);
}

extern "C" esimp::Application_if* sim_register(esimp::MCU_if* obj) {
  mcu = obj;
  systick = mcu->create_timer("systick");
  return &app;
}


extern "C" void simulation_step(int n)
{
  mcu->step(n);
}


extern "C" int main(int argc, char *argv[])
{
  z_cstart();
  return 0;
}





#if defined(CONFIG_THREAD_LOCAL_STORAGE)
uintptr_t z_arm_tls_ptr;
#endif

extern "C" void arch_switch_to_main_thread(struct k_thread *main_thread, char *stack_ptr,
        k_thread_entry_t _main)
{
  _current = main_thread;

#if defined(CONFIG_THREAD_LOCAL_STORAGE)
  z_arm_tls_ptr = main_thread->tls;
#endif

#ifdef CONFIG_INSTRUMENT_THREAD_SWITCHING
  z_thread_mark_switched_in();
#endif


#if defined(CONFIG_BUILTIN_STACK_GUARD)
#error "Built-in PSP limit checks not supported by HW"
#endif /* CONFIG_BUILTIN_STACK_GUARD */

  /* Switch to main thread */
  ThreadInfo *t = static_cast<ThreadInfo *>(main_thread->callee_saved.thread_status);
  t->sim_thread->switch_to();
}




extern "C" void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
         char *stack_ptr, k_thread_entry_t entry,
         void *p1, void *p2, void *p3)
{
  static int count = 0;
  ThreadInfo *t = Z_STACK_PTR_TO_FRAME(ThreadInfo, stack_ptr);
  t->entry = entry;
  t->p1 = p1;
  t->p2 = p2;
  t->p3 = p3;
  t->aborted = 0;
  auto name = std::string("thread") + std::to_string(count);
#if defined(CONFIG_THREAD_NAME)
  if (std::string(thread->name) != "") {
  name = thread->name;
  }
#endif
  t->sim_thread = mcu->create_thread(name.c_str(), t);
  thread->callee_saved.thread_status = t;
}



extern "C" void arch_cpu_idle(void)
{
  /* Somehow irqs are locked at this point by the caller, so unlock first */
  mcu->get_nvic()->set_basepri(0);
  mcu->busy_wait_ns(100e6);
}

extern "C" void arch_cpu_atomic_idle(unsigned int key)
{
  mcu->get_nvic()->set_basepri(key);
  mcu->busy_wait_ns(100e6);
}

extern "C" int arch_swap(unsigned int key)
{
  _current->callee_saved.key = key;
  _current->callee_saved.retval = -EAGAIN;

  /* Switch to new thread */
  struct k_thread *next = _kernel.ready_q.cache;
  ThreadInfo *t_current = static_cast<ThreadInfo *>(_current->callee_saved.thread_status);
  ThreadInfo *t_next = static_cast<ThreadInfo *>(next->callee_saved.thread_status);
// #if defined(CONFIG_THREAD_NAME)
//   t->sim_thread->set_name(_current->name);
// #endif
  _current = next;
  irq_unlock(_current->callee_saved.key);
  if (t_current->aborted) {
    mcu->abort_thread(t_current->sim_thread, t_next->sim_thread);
  } else {
    t_next->sim_thread->switch_to();
  }

  return _current->callee_saved.retval;
}


extern "C" void arch_busy_wait(uint32_t usec_to_wait)
{
  mcu->busy_wait_ns(usec_to_wait*1e3);
}

extern "C" bool systemc_irq_unlocked(unsigned int key)
{
  return key == 0;
}

extern "C" unsigned int systemc_irq_lock(void)
{
  unsigned int ret = mcu->get_nvic()->get_basepri();
  mcu->get_nvic()->set_basepri(_EXC_IRQ_DEFAULT_PRIO);
  return ret;
}

extern "C" void systemc_irq_unlock(unsigned int key)
{
  mcu->get_nvic()->set_basepri(key);
}

extern "C" uint64_t systemc_cycle_get_64(void)
{
  uint64_t t = mcu->get_timestamp_ns();
  return (uint64_t)((t * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) / 1e9);
}

extern "C" void esimp_isr_declare(unsigned int irq_p, int flags, const void *fct, const void *isr_param_p)
{
  auto irq = mcu->get_nvic()->get_irq(irq_p);
  IsrInfo *inf = (IsrInfo*)irq->get_pdata();
  if (inf == nullptr) {
    inf = new IsrInfo();
  }
  inf->fct = (IsrFctPtr)fct;
  inf->param = isr_param_p;
  irq->set_pdata(inf);
}

extern "C" void esimp_irq_priority_set(unsigned int irq_p, unsigned int prio, uint32_t flags)
{
  auto irq = mcu->get_nvic()->get_irq(irq_p);
  irq->set_prio(prio);
}


static void sim_log(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  char txt[1024];
  vsnprintf(txt, sizeof(txt), format, args);
  mcu->log(txt);
  va_end(args);
}


extern "C" void arch_trigger_irq_from_sw(int irq)
{
  mcu->get_nvic()->get_irq(irq)->set_pending_from_sw(true);
}


extern "C" int arch_irq_connect_dynamic(unsigned int irq, unsigned int priority,
			     void (*routine)(const void *parameter),
			     const void *parameter, uint32_t flags)
{
	return irq;
}



extern "C" uint32_t sys_clock_elapsed(void)
{
  return (uint32_t)(systick->get_value_ns() / 1e3);
}


extern "C" int sys_clock_driver_init(const struct device *dev)
{
  systick->stop();
  auto irq = systick->get_irq();
  irq->disable();
  IsrInfo *inf = (IsrInfo*)irq->get_pdata();
  if (inf == nullptr) {
    inf = new IsrInfo();
  }
  inf->fct = systick_isr;
  inf->param = nullptr;
  irq->set_pdata(inf);
  systick->clear_irq();
  irq->set_prio(_EXC_IRQ_DEFAULT_PRIO);
  irq->enable();
  systick->set_mode(esimp::TimerMode::Continuous);
  systick->set_period_ns(100e3);
  systick->start();
  return 0;
}

extern "C" void sys_clock_disable(void)
{
  systick->stop();
}


extern "C" void arch_thread_name_set(struct k_thread *thread)
{
  ThreadInfo *t = static_cast<ThreadInfo *>(thread->callee_saved.thread_status);
#if defined(CONFIG_THREAD_NAME)
  t->sim_thread->set_name(thread->name);
#endif
}


void z_impl_k_thread_abort(k_tid_t thread)
{
  ThreadInfo *t_target = static_cast<ThreadInfo *>(thread->callee_saved.thread_status);
  struct k_thread *current = _current;
  if (_current == thread) {
    if (t_target->aborted == 0) {
      t_target->aborted = 1;
    }
  }

  z_thread_abort(thread);

  ThreadInfo *t_current = static_cast<ThreadInfo *>(_current->callee_saved.thread_status);

  if (t_target->aborted == 0) {
    t_target->aborted = 1;
    mcu->abort_thread(t_target->sim_thread, t_current->sim_thread);
  }
}

extern "C" void systemc_exit(int exit_code)
{
  mcu->exit(esimp::ExitAction::Shutdown, 0);
}


extern "C" bool esimp_is_in_isr(void)
{
  return mcu->is_in_isr();
}


#ifdef CONFIG_IRQ_OFFLOAD
/**
 * Storage for functions offloaded to IRQ
 */


/**
 * IRQ handler for the SW interrupt assigned to irq_offload()
 */
static void offload_sw_irq_handler(const void *a)
{
  ARG_UNUSED(a);
  off_routine(off_parameter);
}

/**
 * @brief Run a function in interrupt context
 *
 * Raise the SW IRQ assigned to handled this
 */
void arch_irq_offload(irq_offload_routine_t routine, const void *parameter)
{
  auto irq = mcu->get_nvic()->get_irq("offload");
  IsrInfo *inf = irq->get_pdata();
  if (inf == nullptr) {
    inf = new IsrInfo();
  }
  inf->fct = routine;
  inf->param = parameter;
  irq->set_pdata(inf);
  irq->enable();
  irq->set_pending_from_sw(true);
}
#endif /* CONFIG_IRQ_OFFLOAD */


extern "C" void arch_irq_disable(unsigned int irq)
{
  mcu->get_nvic()->get_irq(irq)->disable();
}

extern "C" void arch_irq_enable(unsigned int irq)
{
  mcu->get_nvic()->get_irq(irq)->enable();
}

extern "C" int arch_irq_is_enabled(unsigned int irq)
{
  return mcu->get_nvic()->get_irq(irq)->is_enabled();
}



extern "C" int uart_esimp_configure(const struct device *dev, const struct uart_config *cfg)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  c.baudrate = cfg->baudrate;
  c.stop_bits = cfg->stop_bits;
  c.data_bits = cfg->data_bits;
  switch (cfg->parity) {
    case UART_CFG_PARITY_NONE: c.parity = esimp::UART_if::Parity::None; break;
    case UART_CFG_PARITY_ODD: c.parity = esimp::UART_if::Parity::Odd; break;
    case UART_CFG_PARITY_EVEN: c.parity = esimp::UART_if::Parity::Even; break;
    default: return 1; break;
  };
  uart->set_config(c);
  return 0;
}


extern "C" int uart_esimp_poll_in(const struct device *dev, unsigned char *c)
{
  auto uart = mcu->get_uart(dev->name);
  while (!uart->get_status().flags.is_set(esimp::UART_if::Irq::RxFifoNotEmpty)) {
    mcu->busy_wait_ns(10000);
  }
  *c = uart->recv();
	return 0;
}

extern "C" void uart_esimp_poll_out(const struct device *dev,
					unsigned char c)
{
  auto uart = mcu->get_uart(dev->name);
  while (!uart->get_status().flags.is_set(esimp::UART_if::Irq::TxFifoNotFull)) {
    mcu->busy_wait_ns(10000);
  }
  uart->send(c);
}

extern "C" int uart_esimp_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
  auto uart = mcu->get_uart(dev->name);
  int n = 0;
  while ((n < len) && uart->get_status().flags.is_set(esimp::UART_if::Irq::TxFifoNotFull)) {
    uart->send(tx_data[n++]);
  }
  return n;
}

extern "C" int uart_esimp_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
  int n = 0;
  auto uart = mcu->get_uart(dev->name);
  while ((n < size) && uart->get_status().flags.is_set(esimp::UART_if::Irq::RxFifoNotEmpty)) {
    rx_data[n++] = uart->recv();
  }
  return n;
}

extern "C" void uart_esimp_irq_tx_enable(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  c.enabled_irqs.set(esimp::UART_if::Irq::TxFifoNotFull);
  uart->set_config(c);
}

extern "C" void uart_esimp_irq_tx_disable(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  c.enabled_irqs.clear(esimp::UART_if::Irq::TxFifoNotFull);
  uart->set_config(c);
}

extern "C" int uart_esimp_irq_tx_ready(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  return uart->get_status().flags.is_set(esimp::UART_if::Irq::TxFifoNotFull);
}

extern "C" void uart_esimp_irq_rx_enable(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  c.enabled_irqs.set(esimp::UART_if::Irq::RxFifoNotEmpty);
  uart->set_config(c);
}

extern "C" void uart_esimp_irq_rx_disable(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  c.enabled_irqs.clear(esimp::UART_if::Irq::RxFifoNotEmpty);
  uart->set_config(c);
}

extern "C" int uart_esimp_irq_tx_complete(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto status = uart->get_status();
  return status.flags.is_set(esimp::UART_if::Irq::TransferComplete);
}

extern "C" int uart_esimp_irq_rx_ready(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto status = uart->get_status();
  return status.flags.is_set(esimp::UART_if::Irq::RxFifoNotEmpty);
}

extern "C" void uart_esimp_irq_err_enable(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  typedef esimp::UART_if::Irq Irq;
  c.enabled_irqs.set(Irq::FramingError);
  c.enabled_irqs.set(Irq::ParityError);
  c.enabled_irqs.set(Irq::RxFifoOverflow);
  uart->set_config(c);
}

extern "C" void uart_esimp_irq_err_disable(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto c = uart->get_config();
  typedef esimp::UART_if::Irq Irq;
  c.enabled_irqs.clear(Irq::FramingError);
  c.enabled_irqs.clear(Irq::ParityError);
  c.enabled_irqs.clear(Irq::RxFifoOverflow);
  uart->set_config(c);
}

extern "C" int uart_esimp_irq_is_pending(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto status = uart->get_status();
  auto cfg = uart->get_config();
  return (status.flags & cfg.enabled_irqs).is_any_set();
}

extern "C" int uart_esimp_irq_update(const struct device *dev)
{
  /* nothing to do here */
  return 0;
}


extern "C" int uart_esimp_err_check(const struct device *dev)
{
  auto uart = mcu->get_uart(dev->name);
  auto status = uart->get_status();

  typedef esimp::UART_if::Irq Irq;
	uint32_t err = 0U;
  if (status.flags.is_set(Irq::RxFifoOverflow)) {
		err |= UART_ERROR_OVERRUN;
    uart->clear_irq(Irq::RxFifoOverflow);
	}
  if (status.flags.is_set(Irq::ParityError)) {
		err |= UART_ERROR_PARITY;
    uart->clear_irq(Irq::ParityError);
	}
  if (status.flags.is_set(Irq::FramingError)) {
		err |= UART_ERROR_FRAMING;
    uart->clear_irq(Irq::FramingError);
	}

	return err;
}


extern "C" int gpio_esimp_init(const struct device *dev)
{
	return 0;
}

extern "C" int gpio_esimp_configure(const struct device *dev, gpio_pin_t pin,
			     gpio_flags_t flags)
{
  esimp::GpioPin_if *p = mcu->get_gpio_pin(dev->name, pin);
  if (p == NULL) {
    return 1;
  }
  p->configure((enum esimp::GpioPin_if::flags)flags);
	return 0;
}


extern "C" int gpio_esimp_port_get_raw(const struct device *dev, uint32_t *value)
{
	return 0;
}

extern "C" int gpio_esimp_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	return 0;
}

extern "C" int gpio_esimp_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
  for (int i = 0; i < 32; i++) {
    if (pins & (1 << i)) {
      esimp::GpioPin_if *p = mcu->get_gpio_pin(dev->name, i);
      if (p == NULL) {
        return 1;
      }
      p->set();
    }
  }
	return 0;
}

extern "C" int gpio_esimp_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
  for (int i = 0; i < 32; i++) {
    if (pins & (1 << i)) {
      esimp::GpioPin_if *p = mcu->get_gpio_pin(dev->name, i);
      if (p == NULL) {
        return 1;
      }
      p->clear();
    }
  }
	return 0;
}

extern "C" int gpio_esimp_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{
	return 0;
}

extern "C" int gpio_esimp_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	return 0;
}

extern "C" int gpio_esimp_manage_callback(const struct device *dev,
				      struct gpio_callback *callback,
				      bool set)
{
	return 0;
}




#if defined CONFIG_TRACING_SYSTEMC

extern "C" void sys_trace_isr_enter(void)
{
  sim_log("isr_enter");
}

extern "C" void sys_trace_isr_exit(void)
{
  sim_log("isr_enter");
}

extern "C" void sys_trace_isr_exit_to_scheduler(void)
{
  sim_log("isr_enter");
}

extern "C" void sys_trace_idle(void)
{
  sim_log("isr_enter");
}


extern "C" void sys_trace_k_thread_create(struct k_thread *new_thread)
{
  sim_log("k_thread_create 0x%08x %s", new_thread, new_thread->name);
}

extern "C" void sys_trace_k_thread_sleep_enter(k_timeout_t timeout)
{
  sim_log("sleep_enter %d", timeout.ticks);
}

extern "C" void sys_trace_k_thread_msleep_enter(int32_t ms)
{
  sim_log("msleep_enter %d ms", ms);
}

extern "C" void sys_trace_k_thread_usleep_enter(int32_t us)
{
  sim_log("usleep_enter %d us", us);
}

extern "C" void sys_trace_k_thread_busy_wait_enter(int usec_to_wait)
{
  sim_log("busy_wait_enter %d us", usec_to_wait);
}

extern "C" void sys_trace_k_thread_yield(void)
{
  sim_log("thread_yield");
}

extern "C" void sys_trace_k_thread_switched_out(void)
{
  sim_log("thread_switched_out");
}

extern "C" void sys_trace_k_thread_switched_in(void)
{
  sim_log("thread_switched_in");
}

extern "C" void sys_trace_k_thread_ready(struct k_thread *thread)
{
  sim_log("thread_ready %08X", thread);
}

extern "C" void sys_trace_k_thread_pend(struct k_thread *thread)
{
  sim_log("thrad_pend %08X", thread);
}

#endif /* defined CONFIG_TRACING_SYSTEMC */
