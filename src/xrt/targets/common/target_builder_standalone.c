// Copyright 2022-2023, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Standalone driver builder.
 * @author Nathaniel Reilly <nathanmreilly@gmail.com>
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_system.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_config_json.h"

#include "target_builder_helpers.h"
#include "target_builder_interface.h"

#include "standalone/standalone_interface.h"
#include "standalone/standalone_pipeline.h"

#include <assert.h>

/*
 *
 * Helper functions.
 *
 */

static const char *driver_list[] = {
    "standalone",
};


/*
 *
 * Member functions.
 *
 */

static xrt_result_t
standalone_estimate_system(struct xrt_builder *xb,
                          cJSON *config,
                          struct xrt_prober *xp,
                          struct xrt_builder_estimate *estimate)
{
	// The head, the hands and the eyes are all tracked by the headset itself.
	estimate->certain.head = true;
	//! @todo No controllers yet; hands are exposed through the hand tracking roles.
	estimate->certain.left = false;
	estimate->certain.right = false;
	estimate->priority = 1;

	return XRT_SUCCESS;
}

static xrt_result_t
standalone_open_system_impl(struct xrt_builder *xb,
                           cJSON *config,
                           struct xrt_prober *xp,
                           struct xrt_tracking_origin *origin,
                           struct xrt_system_devices *xsysd,
                           struct xrt_frame_context *xfctx,
                           struct t_builder_options *tbo)
{
	/*
	 * Capture and the three trackers. Everything created here is owned by
	 * the frame context, which the system devices tear down after the
	 * devices are gone.
	 */
	struct standalone_pipeline_config pipeline_config;
	standalone_pipeline_fill_default_config(&pipeline_config);

	struct standalone_pipeline *sp = NULL;
	xrt_result_t xret = standalone_pipeline_create(xfctx, &pipeline_config, &sp);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// One device per tracker.
	struct xrt_device *head = standalone_hmd_create(sp->slam);
	if (head == NULL) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	struct xrt_device *hands = standalone_hand_device_create(sp->hand, head);
	struct xrt_device *eyes = standalone_eye_device_create(sp->eye, head);
	if (hands == NULL || eyes == NULL) {
		xrt_device_destroy(&eyes);
		xrt_device_destroy(&hands);
		xrt_device_destroy(&head);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	xsysd->static_xdevs[xsysd->static_xdev_count++] = head;
	xsysd->static_xdevs[xsysd->static_xdev_count++] = hands;
	xsysd->static_xdevs[xsysd->static_xdev_count++] = eyes;

	// Assign to role(s).
	tbo->head = head;
	tbo->eyes = eyes;
	tbo->hand_tracking.unobstructed.left = hands;
	tbo->hand_tracking.unobstructed.right = hands;

	return XRT_SUCCESS;
}

static void
standalone_destroy(struct xrt_builder *xb)
{
	free(xb);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct xrt_builder *
t_builder_standalone_create(void)
{
	struct t_builder *ub = U_TYPED_CALLOC(struct t_builder);

	// xrt_builder fields.
	ub->base.estimate_system = standalone_estimate_system;
	ub->base.open_system = t_builder_open_system_static_roles;
	ub->base.destroy = standalone_destroy;
	ub->base.identifier = "standalone";
	ub->base.name = "Standalone device builder";
	ub->base.driver_identifiers = driver_list;
	ub->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	ub->base.exclude_from_automatic_discovery = false;

	// t_builder fields.
	ub->open_system_static_roles = standalone_open_system_impl;

	return &ub->base;
}
