#include <systemc>

#include "esimp/platform/firmware.hpp"
#include "esimp/platform/mcu.hpp"
#include "esimp/platform/systemc_thread.hpp"
#include "esimp/platform/timer.hpp"
#include "esimp/platform/trace.hpp"
#include "esimp/platform/uart.hpp"
#include "esimp/platform/gpio.hpp"

#include "esimp/systemc/uart_line_half_duplex.hpp"

#include <signal.h>



static void signal_callback_handler(int signum) {
  sc_core::sc_stop();
}


using namespace sc_core;
using namespace sc_dt;


int sc_main(int argc, char* argv[]) {
  if (argc < 3) {
    printf("usage: platform <mcu0-library> <mcu1-library>\n\n");
    return 1;
  }


  esimp::trace::init();

  /* Init MCU0 */
  esimp::MCU mcu0("mcu0");
  esimp::Firmware fw_app0(argv[1]);
  mcu0.add_firmware(&fw_app0);
  {
    esimp::MCU::Configuration cfg;
    cfg.sync_with_realtime = true;
    cfg.f_clk_hz = 100e6;
    cfg.time_quantum_ns = 100e3;
    mcu0.set_configuration(cfg);
  }
  mcu0.add_irq(0, "systick");
  mcu0.add_irq(1, "offload");
  esimp::UART mcu0_uart0("mcu0_uart0");
  mcu0.add_uart(0x25, "uart0", &mcu0_uart0);
  esimp::GpioOutputPin mcu0_tx_en("mcu0_tx_en");
  mcu0.add_gpio_pin(0x26, "gpioa", 0, &mcu0_tx_en);


  /* Init MCU1 */
  esimp::MCU mcu1("mcu1");
  esimp::Firmware fw_app1(argv[2]);
  mcu1.add_firmware(&fw_app1);
  {
    esimp::MCU::Configuration cfg;
    cfg.sync_with_realtime = true;
    cfg.f_clk_hz = 100e6;
    cfg.time_quantum_ns = 100e3;
    mcu1.set_configuration(cfg);
  }
  mcu1.add_irq(0, "systick");
  mcu1.add_irq(1, "offload");
  esimp::UART mcu1_uart0("mcu1_uart0");
  mcu1.add_uart(0x25, "uart0", &mcu1_uart0);
  esimp::GpioOutputPin mcu1_tx_en("mcu1_tx_en");
  mcu1.add_gpio_pin(0x26, "gpioa", 0, &mcu1_tx_en);


  UartLineHalfDuplex line("uart_line");


  sc_signal<bool> tx0_en;
  sc_signal<bool> tx0;
  sc_signal<bool> rx0;
  sc_signal<sc_dt::sc_lv<16> > tx0_dbg;
  sc_signal<sc_dt::sc_lv<16> > rx0_dbg;
  sc_signal<bool> tx0_sample;
  sc_signal<bool> rx0_sample;
  sc_signal<bool> tx1_en;
  sc_signal<bool> tx1;
  sc_signal<bool> rx1;
  sc_signal<sc_dt::sc_lv<16> > tx1_dbg;
  sc_signal<sc_dt::sc_lv<16> > rx1_dbg;
  sc_signal<bool> tx1_sample;
  sc_signal<bool> rx1_sample;
  sc_signal<sc_logic> com;
  sc_signal<bool> conflict;

  mcu0_tx_en.out(tx0_en);
  mcu1_tx_en.out(tx1_en);

  mcu0_uart0.tx(tx0);
  mcu0_uart0.rx(rx0);
  mcu0_uart0.tx_dbg(tx0_dbg);
  mcu0_uart0.rx_dbg(rx0_dbg);
  mcu0_uart0.tx_sample(tx0_sample);
  mcu0_uart0.rx_sample(rx0_sample);
  mcu1_uart0.tx(tx1);
  mcu1_uart0.rx(rx1);
  mcu1_uart0.tx_dbg(tx1_dbg);
  mcu1_uart0.rx_dbg(rx1_dbg);
  mcu1_uart0.tx_sample(tx1_sample);
  mcu1_uart0.rx_sample(rx1_sample);

  line.tx0_en(tx0_en);
  line.tx0(tx0);
  line.rx0(rx0);
  line.tx1_en(tx1_en);
  line.tx1(tx1);
  line.rx1(rx1);
  line.com(com);
  line.conflict(conflict);

  sc_trace_file* file = sc_create_vcd_trace_file("test");

  sc_trace(file, tx0_en, "tx0_en");
  sc_trace(file, tx0, "tx0");
  sc_trace(file, rx0, "rx0");
  sc_trace(file, tx0_dbg, "tx0_dbg");
  sc_trace(file, tx0_sample, "tx0_sample");
  sc_trace(file, rx0_dbg, "rx0_dbg");
  sc_trace(file, rx0_sample, "rx0_sample");
  sc_trace(file, tx1_en, "tx1_en");
  sc_trace(file, tx1, "tx1");
  sc_trace(file, rx1, "rx1");
  sc_trace(file, tx1_dbg, "tx1_dbg");
  sc_trace(file, tx1_sample, "tx1_sample");
  sc_trace(file, rx1_dbg, "rx1_dbg");
  sc_trace(file, rx1_sample, "rx1_sample");
  sc_trace(file, com, "com");
  sc_trace(file, conflict, "conflict");

  signal(SIGINT, signal_callback_handler);

  sc_start();

  std::cout << "close vcd_trace file" << std::endl;
  sc_close_vcd_trace_file(file);


  return 0;
}
