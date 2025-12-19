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

#define USE_DCACHE

#include "postprocess_conf.h"

/* Define sensor orientation */
#define CAMERA_FLIP CMW_MIRRORFLIP_MIRROR

/* Define display size */
#ifdef STM32N6570_DK_REV
#define LCD_BG_WIDTH 800
#define LCD_BG_HEIGHT 480
#else
#define LCD_BG_WIDTH 320
#define LCD_BG_HEIGHT 240
#endif
/* Delay display by DISPLAY_DELAY frame number */
#define DISPLAY_DELAY 1

#ifdef STM32N6570_DK_REV
/* Use yolox */
#define AI_OD_ST_YOLOX_PP_IOU_THRESHOLD             (0.5)
#define AI_OD_ST_YOLOX_PP_CONF_THRESHOLD            (0.6)
#define AI_OD_ST_YOLOX_PP_MAX_BOXES_LIMIT           (10)
#ifndef AI_OD_PP_MAX_BOXES_LIMIT
#define AI_OD_PP_MAX_BOXES_LIMIT                  (AI_OD_ST_YOLOX_PP_MAX_BOXES_LIMIT)
#endif
#else
/* Use yolov2 */
#define AI_OD_YOLOV2_PP_CONF_THRESHOLD            (0.6)
#define AI_OD_YOLOV2_PP_IOU_THRESHOLD             (0.3)
#define AI_OD_YOLOV2_PP_MAX_BOXES_LIMIT           (10)
#ifndef AI_OD_PP_MAX_BOXES_LIMIT
#define AI_OD_PP_MAX_BOXES_LIMIT                  (AI_OD_YOLOV2_PP_MAX_BOXES_LIMIT)
#endif
#endif

#define NN_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP 3
#define NB_CLASSES 2
#define DECLARE_CLASSES_TABLE const char* classes_table[NB_CLASSES] = {\
  "person", "not_person"}

#endif
