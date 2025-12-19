/*---------------------------------------------------------------------------------------------
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *--------------------------------------------------------------------------------------------*/

#ifndef __FD_YUNET_PP_IF_H__
#define __FD_YUNET_PP_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "fd_pp_output_if.h"

/* I/O structures for Yunet post-processing model */
/* ------------------------------------ */
typedef struct
{
  void *pCls_32;
  void *pObj_32;
  void *pBBoxRaw_32;
  void *pKpsRaw_32;
  void *pCls_16;
  void *pObj_16;
  void *pBBoxRaw_16;
  void *pKpsRaw_16;
  void *pCls_8;
  void *pObj_8;
  void *pBBoxRaw_8;
  void *pKpsRaw_8;
} fd_yunet_pp_in_t;


/* Generic Static parameters */
/* ------------------------- */
typedef struct
{
  int32_t   nb_keypoints;
  int32_t   nb_detections_32;
  int32_t   nb_detections_16;
  int32_t   nb_detections_8;
  uint32_t  in_size;
  uint32_t   max_boxes_limit;
  uint32_t allocated_boxes;
  float32_t conf_threshold;
  float32_t iou_threshold;
  int32_t   nb_detect;
  const void *pAnchors_32;
  const void *pAnchors_16;
  const void *pAnchors_8;
  float32_t bbx_32_scale;
  float32_t kps_32_scale;
  float32_t cls_32_scale;
  float32_t obj_32_scale;
  float32_t bbx_16_scale;
  float32_t kps_16_scale;
  float32_t cls_16_scale;
  float32_t obj_16_scale;
  float32_t bbx_8_scale;
  float32_t kps_8_scale;
  float32_t cls_8_scale;
  float32_t obj_8_scale;
  int8_t cls_32_zero_point;
  int8_t obj_32_zero_point;
  int8_t bbx_32_zero_point;
  int8_t kps_32_zero_point;
  int8_t cls_16_zero_point;
  int8_t obj_16_zero_point;
  int8_t bbx_16_zero_point;
  int8_t kps_16_zero_point;
  int8_t cls_8_zero_point;
  int8_t obj_8_zero_point;
  int8_t bbx_8_zero_point;
  int8_t kps_8_zero_point;
} fd_yunet_pp_static_param_t;


/* Exported functions ------------------------------------------------------- */

/*!
 * @brief Resets face detection yunet post processing
 *
 * @param [IN] Input static parameters
 * @retval Error code
 */
int32_t fd_yunet_pp_reset(fd_yunet_pp_static_param_t *pInput_static_param);

/*!
 * @brief Face detector post processing : includes output detector remapping,
 *        nms and score filtering for fd yunet quantized signed inputs
 *
 * @param [IN] Pointer on input structure pointing data
 *             Pointer on output structure pointing data
 *             Pointer on static parameters
 * @retval Error code
 */
int32_t fd_yunet_pp_process_int8(fd_yunet_pp_in_t *pInput,
                                 fd_pp_out_t *pOutput,
                                 fd_yunet_pp_static_param_t *pInput_static_param);

/*!
 * @brief Face detector post processing : includes output detector remapping,
 *        nms and score filtering for fd yunet
 *
 * @param [IN] Pointer on input structure pointing data
 *             Pointer on output structure pointing data
 *             Pointer on static parameters
 * @retval Error code
 */
int32_t fd_yunet_pp_process(fd_yunet_pp_in_t *pInput,
                            fd_pp_out_t *pOutput,
                            fd_yunet_pp_static_param_t *pInput_static_param);
#ifdef __cplusplus
 }
#endif

#endif      /* __FD_YUNET_PP_IF_H__  */
