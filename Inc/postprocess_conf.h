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

#include "arm_math.h"

#ifdef STM32N6570_DK_REV
#define POSTPROCESS_TYPE                          POSTPROCESS_OD_ST_YOLOX_UF

#define AI_OD_ST_YOLOX_PP_NB_CLASSES                (1)
#define AI_OD_ST_YOLOX_PP_L_GRID_WIDTH              (52)
#define AI_OD_ST_YOLOX_PP_L_GRID_HEIGHT             (52)
#define AI_OD_ST_YOLOX_PP_M_GRID_WIDTH              (26)
#define AI_OD_ST_YOLOX_PP_M_GRID_HEIGHT             (26)
#define AI_OD_ST_YOLOX_PP_S_GRID_WIDTH              (13)
#define AI_OD_ST_YOLOX_PP_S_GRID_HEIGHT             (13)
#define AI_OD_ST_YOLOX_PP_NB_ANCHORS                (3)

/* Anchor boxes */
static const float32_t AI_OD_ST_YOLOX_PP_L_ANCHORS[2 * AI_OD_ST_YOLOX_PP_NB_ANCHORS] = {26.000000, 26.000000, 3.640000, 13.000000, 11.960000, 36.40000};
static const float32_t AI_OD_ST_YOLOX_PP_M_ANCHORS[2 * AI_OD_ST_YOLOX_PP_NB_ANCHORS] = {13.000000, 13.000000, 1.820000,  6.500000,  5.980000, 18.20000};
static const float32_t AI_OD_ST_YOLOX_PP_S_ANCHORS[2 * AI_OD_ST_YOLOX_PP_NB_ANCHORS] = { 6.500000,  6.500000, 0.910000,  3.250000,  2.990000,  9.10000};
#else
#define POSTPROCESS_TYPE                          POSTPROCESS_OD_YOLO_V2_UF

#define AI_OD_YOLOV2_PP_NB_CLASSES                (1)
#define AI_OD_YOLOV2_PP_NB_ANCHORS                (5)
#define AI_OD_YOLOV2_PP_GRID_WIDTH                (7)
#define AI_OD_YOLOV2_PP_GRID_HEIGHT               (7)
#define AI_OD_YOLOV2_PP_NB_INPUT_BOXES            (AI_OD_YOLOV2_PP_GRID_WIDTH * AI_OD_YOLOV2_PP_GRID_HEIGHT)
/* Anchor boxes */
static const float32_t AI_OD_YOLOV2_PP_ANCHORS[2*AI_OD_YOLOV2_PP_NB_ANCHORS] = {
    0.9883000000f,     3.3606000000f,
    2.1194000000f,     5.3759000000f,
    3.0520000000f,     9.1336000000f,
    5.5517000000f,     9.3066000000f,
    9.7260000000f,     11.1422000000f,
  };
#endif

#endif      /* __POSTPROCESS_CONF_H__  */
