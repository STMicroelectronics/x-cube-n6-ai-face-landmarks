/**
 ******************************************************************************
 * @file    app_config.h
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef APP_CONFIG
#define APP_CONFIG

#include "postprocess_conf.h"

#define USE_DCACHE

/* Define sensor orientation */
#define CAMERA_FLIP CMW_MIRRORFLIP_MIRROR

/* Define display size */
#ifdef STM32N6570_DK_REV
#define LCD_BG_WIDTH 800
#define LCD_BG_HEIGHT 480
/* Delay display by DISPLAY_DELAY frame number */
#define DISPLAY_DELAY 1
#else
#ifdef SCR_LIB_USE_SPI
#define LCD_BG_WIDTH 320
#define LCD_BG_HEIGHT 240
#else
#define LCD_BG_WIDTH 320
#define LCD_BG_HEIGHT 200
#endif
#define DISPLAY_DELAY 0
#endif

#endif
