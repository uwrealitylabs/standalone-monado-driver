// Copyright 2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  What OpenXR defines are enabled in this oxr build.
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

/* #undef XR_USE_GRAPHICS_API_D3D11 */
/* #undef XR_USE_GRAPHICS_API_D3D12 */
#define XR_USE_GRAPHICS_API_OPENGL
#define XR_USE_GRAPHICS_API_OPENGL_ES
#define XR_USE_GRAPHICS_API_VULKAN
/* #undef XR_USE_PLATFORM_ANDROID */
#define XR_USE_PLATFORM_EGL
/* #undef XR_USE_PLATFORM_WIN32 */
#define XR_USE_PLATFORM_XLIB
