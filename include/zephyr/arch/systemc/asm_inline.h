

#ifndef ZEPHYR_INCLUDE_ARCH_SYSTEMC_ASM_INLINE_H_
#define ZEPHYR_INCLUDE_ARCH_SYSTEMC_ASM_INLINE_H_

/*
 * The file must not be included directly
 * Include kernel.h instead
 */

#if defined(__GNUC__)
#include <zephyr/arch/systemc/asm_inline_gcc.h>
#else
#error "Only a compiler with GNU C extensions is supported for the SYSTEMC arch"
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_SYSTEMC_ASM_INLINE_H_ */
