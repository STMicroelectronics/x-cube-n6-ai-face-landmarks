/**
  ******************************************************************************
  * @file    ld.h
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

#ifndef _FACE_LANDMARK_
#define _FACE_LANDMARK_

#include "stai_face_landmark.h"

/* model hard coded values */
#define LD_WIDTH              ((float)STAI_FACE_LANDMARK_IN_1_WIDTH)
#define LD_HEIGHT             ((float)STAI_FACE_LANDMARK_IN_1_HEIGHT)
#define LD_LANDMARK_NB        468
#define LD_BINDING_NB         0

typedef struct {
  float x;
  float y;
} ld_point_t;

void ld_post_process(float *raw_landmarks, ld_point_t landmarks[LD_LANDMARK_NB]);

#endif
