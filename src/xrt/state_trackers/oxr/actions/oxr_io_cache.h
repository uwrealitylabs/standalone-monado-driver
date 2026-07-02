// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Per-session snapshot wrapper around @ref xrt_io_cache.
 * @ingroup oxr_main
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_io_cache.h"

#include "oxr_defines.h"
#include "oxr_extension_support.h"
#include "oxr_forward_declarations.h"


#ifdef __cplusplus
extern "C" {
#endif

struct xrt_system_devices;
struct xrt_space_overseer;

/*!
 * Per session-action-context snapshot of all device inputs and outputs, wrapping
 * an @ref xrt_io_cache.
 *
 * One of these lives on every @ref oxr_session_action_context. It is the seam
 * that hides the underlying @ref xrt_device instances from the OpenXR action
 * code: resolved action bindings (@ref oxr_action_input / @ref oxr_action_output)
 * point directly into the @ref inputs / @ref outputs arrays here, and those
 * arrays are refreshed once per xrSyncActions from the backing devices in a
 * single call.
 *
 * The @ref inputs and @ref outputs arrays are owned here and sized to match the
 * device layout described by @ref info. Because action bindings hold raw
 * pointers into these arrays, the arrays must not move while bindings are live:
 * when the device set changes the arrays are reallocated (see
 * @ref oxr_io_cache_recreate) as an explicit step, bracketed by tearing down and
 * re-resolving the bindings. The backing @ref xic stays alive across that.
 */
struct oxr_io_cache
{
	//! Backing cache object that reads from / writes to the devices.
	struct xrt_io_cache *xic;

	//! Layout (device table, counts and generation) of the current snapshot.
	struct xrt_io_cache_info info;

	//! Snapshot arrays, owned here and sized per @ref info; bindings point into these.
	struct xrt_input *inputs;
	struct xrt_output *outputs;
};

/*!
 * Create the cache. Call once, when the owning session action context is
 * created.
 *
 * @public @memberof oxr_io_cache
 */
XRT_CHECK_RESULT XrResult
oxr_io_cache_init(struct oxr_io_cache *cache, struct xrt_system_devices *xsysd, struct xrt_space_overseer *xso);

/*!
 * Destroy the cache. Call once, when the owning session action context is
 * destroyed.
 *
 * @public @memberof oxr_io_cache
 */
void
oxr_io_cache_fini(struct oxr_io_cache *cache);

/*!
 * Recreate the snapshot for the current device layout: re-query @ref info and
 * reallocate the @ref inputs / @ref outputs arrays. The backing @ref xic is
 * kept alive.
 *
 * Call this as an explicit step after @ref oxr_io_cache_sync reports the cache
 * is stale, and only after the action bindings that point into the old arrays
 * have been torn down; rebind afterwards so bindings point into the new arrays.
 *
 * @public @memberof oxr_io_cache
 */
XRT_CHECK_RESULT XrResult
oxr_io_cache_recreate(struct oxr_logger *log, struct oxr_io_cache *cache);

/*!
 * Refresh the snapshot: read all device inputs and outputs into @ref inputs /
 * @ref outputs with a single call. Call once per xrSyncActions.
 *
 * This never reallocates. If the device layout changed since the arrays were
 * sized the read cannot be satisfied, @p out_stale is set to true and the
 * caller must tear down the bindings, @ref oxr_io_cache_recreate, rebind, and
 * sync again. On a normal refresh @p out_stale is set to false.
 *
 * @public @memberof oxr_io_cache
 */
XRT_CHECK_RESULT XrResult
oxr_io_cache_sync(struct oxr_logger *log, struct oxr_io_cache *cache, bool *out_stale);


#ifdef __cplusplus
}
#endif
