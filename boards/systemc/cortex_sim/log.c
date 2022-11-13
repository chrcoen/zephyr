

#include <stdio.h>  /* for printfs */
#include <stdarg.h> /* for va args */

void posix_print_trace(const char *format, ...)
{
	va_list variable_args;

	va_start(variable_args, format);
	vfprintf(stdout, format, variable_args);
	va_end(variable_args);
}


int arch_printk_char_out(int c)
{
  putchar(c);
  return 0;
}
