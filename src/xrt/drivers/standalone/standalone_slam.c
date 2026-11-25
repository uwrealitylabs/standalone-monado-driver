// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SLAM tracker implementation table.
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "standalone_slam.h"

#include <string.h>


struct slam_impl
{
	const char *name;
	standalone_slam_create_fn create;
};

static const struct slam_impl slam_impls[] = {
    {"null", standalone_slam_null_create},
#ifdef XRT_FEATURE_SLAM
    {"t_slam", standalone_slam_t_slam_create},
#endif
};

static const struct slam_impl *
find_impl(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(slam_impls); i++) {
		if (strcmp(slam_impls[i].name, name) == 0) {
			return &slam_impls[i];
		}
	}
	return NULL;
}

bool
standalone_slam_has_impl(const char *name)
{
	return name != NULL && find_impl(name) != NULL;
}

xrt_result_t
standalone_slam_create(struct xrt_frame_context *xfctx,
                       const struct standalone_slam_config *config,
                       struct standalone_slam_tracker **out_sst)
{
	const char *name = config->impl != NULL ? config->impl : "null";

	const struct slam_impl *impl = find_impl(name);
	if (impl == NULL) {
		U_LOG_E("No Standalone SLAM tracker implementation named '%s' in this build", name);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	*out_sst = NULL;

	xrt_result_t xret = impl->create(xfctx, config, out_sst);
	if (xret != XRT_SUCCESS || *out_sst == NULL) {
		U_LOG_E("Failed to create Standalone SLAM tracker implementation '%s'", name);
		return xret != XRT_SUCCESS ? xret : XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	U_LOG_I("Standalone SLAM tracker: '%s' (cameras 0x%02x, IMUs 0x%x)", name,
	        standalone_sinks_cam_mask(&(*out_sst)->sinks), standalone_sinks_imu_mask(&(*out_sst)->sinks));

	return XRT_SUCCESS;
}
