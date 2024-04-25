/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash/ra_flash_api_extensions.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_USERSPACE
#include <zephyr/syscall.h>
#include <zephyr/syscall_handler.h>
#endif

#include <soc.h>
#include "flash_ra.h"

#if defined(CONFIG_FLASH_RA_WRITE_PROTECT)
int flash_ra_ex_op_write_protect(const struct device *dev, const uintptr_t in, void *out)
{
	const struct flash_ra_ex_write_protect_in *request =
		(const struct flash_ra_ex_write_protect_in *)in;
	struct flash_ra_ex_write_protect_out *result = (struct flash_ra_ex_write_protect_out *)out;

	int rc = 0, rc2 = 0;
#ifdef CONFIG_USERSPACE
	bool syscall_trap = z_syscall_trap();
#endif

	if (request != NULL) {
#ifdef CONFIG_USERSPACE
		struct flash_ra_ex_write_protect_in in_copy;

		if (syscall_trap) {
			Z_OOPS(z_user_from_copy(&in_copy, request, sizeof(in_copy)));
			request = &in_copy;
		}
#endif

		/* if both enable and disable are set  */
		if ((request->protect_enable.BPS[0] & request->protect_disable.BPS[0]) ||
		    (request->protect_enable.BPS[1] & request->protect_disable.BPS[1]) ||
		    (request->protect_enable.BPS[2] & request->protect_disable.BPS[2]) ||
		    (request->protect_enable.BPS[3] & request->protect_disable.BPS[3])) {
			return EINVAL;
		}

		rc = flash_ra_block_protect_set(dev, request);
	}

	if (result != NULL) {
#ifdef CONFIG_USERSPACE
		struct flash_ra_ex_write_protect_out out_copy;

		if (syscall_trap) {
			result = &out_copy;
		}
#endif
		rc2 = flash_ra_block_protect_get(dev, result);
		if (!rc) {
			rc = rc2;
		}

#ifdef CONFIG_USERSPACE
		if (syscall_trap) {
			Z_OOPS(z_user_to_copy(out, result, sizeof(out_copy)));
		}
#endif
	}

	return rc;
}
#endif /* CONFIG_FLASH_RA_WRITE_PROTECT */
