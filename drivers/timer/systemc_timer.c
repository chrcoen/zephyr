#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>


extern int sys_clock_driver_init(const struct device *dev);

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
