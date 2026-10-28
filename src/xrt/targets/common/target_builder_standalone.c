// Copyright 2022-2023, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Standalone driver builder.
 * @author Nathaniel Reilly <nathanmreilly@gmail.com>
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_system.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_config_json.h"

#include "target_builder_helpers.h"
#include "target_builder_interface.h"

#include "standalone/sample_interface.h"

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
	estimate->certain.head = true;
	estimate->certain.left = true;
	estimate->certain.right = true;
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
	struct xrt_device *head = standalone_create();

	// Make the objects be tracked in space.
	//! @todo Make these be a option to the hmd create function, or just make it be there from the start.
	//head->supported.orientation_tracking = true;
	//head->supported.position_tracking = true;
	//! @todo Create a shared tracking origin on the system devices struct instead.
	//head->tracking_origin->type = XRT_TRACKING_TYPE_OTHER; // Just anything other then none.

	// Assign to role(s).
	tbo->head = head;

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
