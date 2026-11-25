// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hand tracker implementation table.
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "standalone_hand.h"

#include <string.h>


struct hand_impl
{
	const char *name;
	standalone_hand_create_fn create;
};

static const struct hand_impl hand_impls[] = {
    {"null", standalone_hand_null_create},
};

static const struct hand_impl *
find_impl(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(hand_impls); i++) {
		if (strcmp(hand_impls[i].name, name) == 0) {
			return &hand_impls[i];
		}
	}
	return NULL;
}

bool
standalone_hand_has_impl(const char *name)
{
	return name != NULL && find_impl(name) != NULL;
}

xrt_result_t
standalone_hand_create(struct xrt_frame_context *xfctx,
                       const struct standalone_hand_config *config,
                       struct standalone_hand_tracker **out_sht)
{
	const char *name = config->impl != NULL ? config->impl : "null";

	const struct hand_impl *impl = find_impl(name);
	if (impl == NULL) {
		U_LOG_E("No Standalone hand tracker implementation named '%s' in this build", name);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	*out_sht = NULL;

	xrt_result_t xret = impl->create(xfctx, config, out_sht);
	if (xret != XRT_SUCCESS || *out_sht == NULL) {
		U_LOG_E("Failed to create Standalone hand tracker implementation '%s'", name);
		return xret != XRT_SUCCESS ? xret : XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	U_LOG_I("Standalone hand tracker: '%s' (cameras 0x%02x)", name, standalone_sinks_cam_mask(&(*out_sht)->sinks));

	return XRT_SUCCESS;
}
