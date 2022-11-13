
#include <zephyr/logging/log_backend.h>

static void sync_string(const struct log_backend *const backend,
		     struct log_msg_ids src_level, uint32_t timestamp,
		     const char *fmt, va_list ap)
{
}

static void panic(struct log_backend const *const backend)
{
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
}

static void sync_hexdump(const struct log_backend *const backend,
			 struct log_msg_ids src_level, uint32_t timestamp,
			 const char *metadata, const uint8_t *data, uint32_t length)
{
}

const struct log_backend_api log_backend_native_posix_api = {
	.process = NULL,
	.put = NULL,
	.put_sync_string = sync_string,
	.put_sync_hexdump = sync_hexdump,
	.panic = panic,
	.dropped = NULL,
};

