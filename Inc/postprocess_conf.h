/*---------------------------------------------------------------------------------------------
#  * Copyright (c) 2023 STMicroelectronics.
#  * All rights reserved.
#  *
#  * This software is licensed under terms that can be found in the LICENSE file in
#  * the root directory of this software component.
#  * If no LICENSE file comes with this software, it is provided AS-IS.
#  *--------------------------------------------------------------------------------------------*/

/* ---------------    Generated code    ----------------- */
#ifndef __POSTPROCESS_CONF_H__
#define __POSTPROCESS_CONF_H__


#ifdef __cplusplus
  extern "C" {
#endif

#include "arm_math.h"

/* Select face detector postprocess */
#define POSTPROCESS_TYPE POSTPROCESS_FD_BLAZEFACE_UI

/* I/O configuration */
#define AI_FD_BLAZEFACE_PP_NB_KEYPOINTS      (6)
#define AI_FD_BLAZEFACE_PP_NB_CLASSES        (1)
#define AI_FD_BLAZEFACE_PP_IMG_SIZE          (128)
#define AI_FD_BLAZEFACE_PP_OUT_0_NB_BOXES    (512)
#define AI_FD_BLAZEFACE_PP_OUT_1_NB_BOXES    (384)

/* --------  Tuning below can be modified by the application --------- */
#define AI_FD_BLAZEFACE_PP_MAX_BOXES_LIMIT   (3)
#define AI_FD_BLAZEFACE_PP_CONF_THRESHOLD    (0.8)
#define AI_FD_BLAZEFACE_PP_IOU_THRESHOLD     (0.5)

#endif      /* __POSTPROCESS_CONF_H__  */

