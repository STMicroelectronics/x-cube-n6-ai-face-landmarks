/**
  ******************************************************************************
  * @file    app.h
  * @author  MDG Application Team
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef APP_H
#define APP_H

#include "postprocess_conf.h"

/* Model Related Info */
#define NN_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1

/* Display config */
#if HAS_ROTATION_SUPPORT == 1
#define DISPLAY_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_ARGB8888
#define DISPLAY_BPP 4
#else
#define DISPLAY_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define DISPLAY_BPP 3
#endif

void app_run(void);

#endif