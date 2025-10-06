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
#ifndef DISPLAY_HELPERS
#define DISPLAY_HELPERS

#include <stm32n6xx_hal.h>
#include <nema_core.h>
#include <assert.h>

#define RESX 800
#define RESY 480

#define FB_MEM_POOL_ID      1
#define STENCIL_MEM_POOL_ID 2
#define ASSET_MEM_POOL_ID   3

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes framebuffer(s) and LTDC.
 * @param uint32_t pixel format (defined in nema_graphics.h).
 * @param uint8_t number of framebuffers.
 * @retval int OK.
 */
int init_display(uint32_t format, uint8_t count);

/**
 * @brief Swaps the framebuffer for LTDC.
 * @retval None 
 */
void swap_buffers(void);

/**
 * @brief Get index of current framebuffer.
 * @retval current framebuffer index.
 */
uint32_t get_framebuffer_idx(void);

/**
 * @brief Get number of framebuffers.
 * @retval framebuffer_count
 */
uint32_t get_framebuffers_count();

/**
 * @brief Returns the framebuffer not currently in use. 
 * @retval img_obj_t framebuffer.
 */
img_obj_t *get_current_framebuffer(void);

/**
 * @brief Returns chosen framebuffer.
 * @param uint8_t index (0 or 1)
 * @retval img_obj_t framebuffer.
 */
img_obj_t *get_framebuffer(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif
