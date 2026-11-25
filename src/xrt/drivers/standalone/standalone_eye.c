// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Eye tracker implementation table.
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "standalone_eye.h"

#include <string.h>


struct eye_impl
{
	const char *name;
	standalone_eye_create_fn create;
};

static const struct eye_impl eye_impls[] = {
    {"null", standalone_eye_null_create},
};

static const struct eye_impl *
find_impl(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(eye_impls); i++) {
		if (strcmp(eye_impls[i].name, name) == 0) {
			return &eye_impls[i];
		}
	}
	return NULL;
}

bool
standalone_eye_has_impl(const char *name)
{
	return name != NULL && find_impl(name) != NULL;
}

xrt_result_t
standalone_eye_create(struct xrt_frame_context *xfctx,
                      const struct standalone_eye_config *config,
                      struct standalone_eye_tracker **out_set)
{
	const char *name = config->impl != NULL ? config->impl : "null";

	const struct eye_impl *impl = find_impl(name);
	if (impl == NULL) {
		U_LOG_E("No Standalone eye tracker implementation named '%s' in this build", name);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	*out_set = NULL;

	xrt_result_t xret = impl->create(xfctx, config, out_set);
	if (xret != XRT_SUCCESS || *out_set == NULL) {
		U_LOG_E("Failed to create Standalone eye tracker implementation '%s'", name);
		return xret != XRT_SUCCESS ? xret : XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	U_LOG_I("Standalone eye tracker: '%s' (cameras 0x%02x)", name, standalone_sinks_cam_mask(&(*out_set)->sinks));

	return XRT_SUCCESS;
}
