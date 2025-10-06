/**
  ******************************************************************************
  * Copyright (c) 2020(-2022) STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/**
  ******************************************************************************
  * @file    platform_pool_config.h
  * @author  MCD Application Team
  * @brief   Header file of NemaGFX Memory Pools Definitions for STM32 Platforms.
  *          This file provides definition for memory pools (ids, sizes and
  *          addresses) definitions being used by the NemaGFX TSi Allocator.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PLATFORM_POOL_CONFIG_H__
#define PLATFORM_POOL_CONFIG_H__

#include "nema_display.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct
{
    uint32_t base_addr;
    uint32_t mem_size;
    uint32_t allocated;
} nema_mem_pool_desc_t;

/* Custom Memory Pool */
#if defined(HAS_CUSTOM_MEM_POOLS_CONFIG) && (HAS_CUSTOM_MEM_POOLS_CONFIG == 1)
#include "app_custom_pools_config.h"
#else

/* Build Flags        */
#define NEMA_USE_TSI_MALLOC             /* Use TSi Memory Allocator                         */
#define NEMA_MULTI_MEM_POOLS            /* Use Memory Pools - require NEMA_USE_TSI_MALLOC   */

/* Sanity Checks      */
#if defined(NEMA_MULTI_MEM_POOLS) && !defined(NEMA_USE_TSI_MALLOC)
#error Please implement your own memory pools manager and create pools or NEMA_USE_TSI_MALLOC!
#endif

/* ----------------------------------------------------------------------------- */
/* Graphics Memory Mapping                                                       */
/* ----------------------------------------------------------------------------- */
/* NEMA_MULTI_MEM_POOLS_CNT pools must be equal or less than 4                   */
#define NEMA_MULTI_MEM_POOLS_CNT	3

#define PSRAM_BASE_ADDR             0x90000000U

#if 1
/* Pool 0 - NemaGFX Command Lists */
#define NEMA_MEM_POOL_CL            0
#define GFX_MEM_CL_SIZE             (512 * 1024)
#define GFX_MEM_CL_BASEADDR         0x34180000

/* Pool 1 - Framebuffer(s) */
#define NEMA_MEM_POOL_FB            FB_MEM_POOL_ID
#define GFX_MEM_FB_SIZE             RESX * RESY * 4 * 2 + 0x200
#define GFX_MEM_FB_BASEADDR         PSRAM_BASE_ADDR

/* Pool 2 - Stencil Buffer */
#define NEMA_MEM_POOL_VG            STENCIL_MEM_POOL_ID
#define GFX_MEM_VG_SIZE             RESX * RESY + 0x100
#define GFX_MEM_VG_BASEADDR         GFX_MEM_FB_BASEADDR + GFX_MEM_FB_SIZE

#else
/* EXTERNAL FRAMEBUFFERS IN PSRAM */
#if    defined(CL_FB_EXTERNAL)
/* Pool 0 - NemaGFX Command Lists */
#define NEMA_MEM_POOL_CL            0
#define GFX_MEM_CL_SIZE             (512 * 1024)
#define GFX_MEM_CL_BASEADDR         0x34340000U

/* Pool 1 - Framebuffer(s) */
#define NEMA_MEM_POOL_FB            FB_MEM_POOL_ID
#define GFX_MEM_FB_SIZE             RESX * RESY * 4 * 2 + 0x200
#define GFX_MEM_FB_BASEADDR         PSRAM_BASE_ADDR

/* Pool 2 - Stencil Buffer */
#define NEMA_MEM_POOL_VG            STENCIL_MEM_POOL_ID
#define GFX_MEM_VG_SIZE             RESX * RESY + 0x100
#define GFX_MEM_VG_BASEADDR         GFX_MEM_FB_BASEADDR + GFX_MEM_FB_SIZE

/* INTERNAL FRAMEBUFFERS IN AXISRAM */
#elif   defined(CL_FB_INTERNAL)
/* Pool 0 - NemaGFX Command Lists */
#define NEMA_MEM_POOL_CL            0
#define GFX_MEM_CL_SIZE             (512 * 1024)
#define GFX_MEM_CL_BASEADDR         0x340CFFFFU

/* Pool 1 - Framebuffer(s) */
#define NEMA_MEM_POOL_FB            FB_MEM_POOL_ID
#define GFX_MEM_FB_SIZE             RESX * RESY * 4 + 0x200
#define GFX_MEM_FB_BASEADDR         GFX_MEM_CL_BASEADDR + GFX_MEM_CL_SIZE

/* Pool 2 - Stencil Buffer */
#define NEMA_MEM_POOL_VG            STENCIL_MEM_POOL_ID
#define GFX_MEM_VG_SIZE             RESX * RESY + 0x100
#define GFX_MEM_VG_BASEADDR         GFX_MEM_FB_BASEADDR + GFX_MEM_FB_SIZE

#else
#error "Either CL_FB_EXTERNAL or CL_FB_INTERNAL must be defined"
#endif
#endif

static nema_mem_pool_desc_t memPools[NEMA_MULTI_MEM_POOLS_CNT] =
{
    {GFX_MEM_CL_BASEADDR    , GFX_MEM_CL_SIZE     , 0 }, /* Pool 0 */
    {GFX_MEM_FB_BASEADDR    , GFX_MEM_FB_SIZE     , 0 }, /* Pool 1 */
    {GFX_MEM_VG_BASEADDR    , GFX_MEM_VG_SIZE     , 0 }, /* Pool 2 */
};

#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PLATFORM_POOL_CONFIG_H__ */