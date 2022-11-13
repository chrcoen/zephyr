
#ifndef ZEPHYR_INCLUDE_ARCH_SYSTEMC_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_SYSTEMC_ARCH_H_

/* Add include for DTS generated information */
#include <zephyr/devicetree.h>

#include <zephyr/toolchain.h>
#include <zephyr/irq.h>
#include <zephyr/arch/systemc/asm_inline.h>
#include <zephyr/arch/systemc/thread.h>
#include <zephyr/sw_isr_table.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_64BIT
#define ARCH_STACK_PTR_ALIGN 8
#else
#define ARCH_STACK_PTR_ALIGN 4
#endif

bool systemc_irq_unlocked(unsigned int key);
unsigned int systemc_irq_lock(void);
void systemc_irq_unlock(unsigned int key);
uint64_t systemc_cycle_get_64(void);
void systemc_exit(int exit_code);

void esimp_isr_declare(unsigned int irq_p, int flags, const void *,
		       const void *isr_param_p);
void esimp_irq_priority_set(unsigned int irq, unsigned int prio,
			    uint32_t flags);



struct __esf {
	uint32_t dummy; /*maybe we will want to add something someday*/
};

typedef struct __esf z_arch_esf_t;


static inline uint32_t arch_k_cycle_get_32(void)
{
	return (uint32_t)systemc_cycle_get_64();
}

static inline uint64_t arch_k_cycle_get_64(void)
{
	return systemc_cycle_get_64();
}

static ALWAYS_INLINE void arch_nop(void)
{
	__asm__ volatile("nop");
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return systemc_irq_unlocked(key);
}

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	return systemc_irq_lock();
}


static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
	systemc_irq_unlock(key);
}


/**
 * Configure a static interrupt.
 *
 * @param irq_p IRQ line number
 * @param priority_p Interrupt priority
 * @param isr_p Interrupt service routine
 * @param isr_param_p ISR parameter
 * @param flags_p IRQ options
 */
#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
{ \
	esimp_isr_declare(irq_p, 0, (void (*)(const void *))isr_p, isr_param_p); \
	esimp_irq_priority_set(irq_p, priority_p, flags_p); \
}


/**
 * Configure a 'direct' static interrupt.
 *
 * See include/irq.h for details.
 */
#define ARCH_IRQ_DIRECT_CONNECT(irq_p, priority_p, isr_p, flags_p) \
{ \
	esimp_isr_declare(irq_p, ISR_FLAG_DIRECT, \
			  (void (*)(const void *))isr_p, NULL); \
	esimp_irq_priority_set(irq_p, priority_p, flags_p); \
}

/**
 * POSIX Architecture (board) specific ISR_DIRECT_DECLARE(),
 * See include/irq.h for more information.
 *
 * The return of "name##_body(void)" is the indication of the interrupt
 * (maybe) having caused a kernel decision to context switch
 *
 * Note that this convention is changed relative to the ARM and x86 archs
 *
 * All pre/post irq work of the interrupt is handled in the board
 * posix_irq_handler() both for direct and normal interrupts together
 */
#define ARCH_ISR_DIRECT_DECLARE(name) \
	static inline int name##_body(void); \
	int name(void) \
	{ \
		int check_reschedule; \
		check_reschedule = name##_body(); \
		return check_reschedule; \
	} \
	static inline int name##_body(void)

#define ARCH_ISR_DIRECT_HEADER()   do { } while (0)
#define ARCH_ISR_DIRECT_FOOTER(a)  do { } while (0)



#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_SYSTEMC_ARCH_H_ */
