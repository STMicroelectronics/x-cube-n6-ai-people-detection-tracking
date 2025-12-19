 /**
 ******************************************************************************
 * @file    app_postprocess_fd_blazeface_ui.c
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


#include "app_postprocess.h"
#include "app_config.h"
#include <assert.h>

#if POSTPROCESS_TYPE == POSTPROCESS_FD_BLAZEFACE_UI
#include "fd_blazeface_anchors_0.h"
#include "fd_blazeface_anchors_1.h"
#define AI_FD_BLAZEFACE_PP_TOTAL_BOXES (AI_FD_BLAZEFACE_PP_OUT_0_NB_BOXES + AI_FD_BLAZEFACE_PP_OUT_1_NB_BOXES)
static fd_pp_outBuffer_t out_detections[AI_FD_BLAZEFACE_PP_TOTAL_BOXES];
static fd_pp_keyPoints_t out_keyPoints[AI_FD_BLAZEFACE_PP_TOTAL_BOXES * AI_FD_BLAZEFACE_PP_NB_KEYPOINTS];

int32_t app_postprocess_init(void *params_postprocess, stai_network_info *NN_Info)
{
  int32_t error = AI_FD_PP_ERROR_NO;
  fd_blazeface_pp_static_param_t *params = (fd_blazeface_pp_static_param_t *) params_postprocess;

  params->in_size            = AI_FD_BLAZEFACE_PP_IMG_SIZE;
  params->nb_classes         = AI_FD_BLAZEFACE_PP_NB_CLASSES;
  params->nb_keypoints       = AI_FD_BLAZEFACE_PP_NB_KEYPOINTS;
  params->nb_detections_0    = AI_FD_BLAZEFACE_PP_OUT_0_NB_BOXES;
  params->nb_detections_1    = AI_FD_BLAZEFACE_PP_OUT_1_NB_BOXES;
  params->max_boxes_limit    = AI_FD_BLAZEFACE_PP_MAX_BOXES_LIMIT;
  params->conf_threshold     = AI_FD_BLAZEFACE_PP_CONF_THRESHOLD;
  params->iou_threshold      = AI_FD_BLAZEFACE_PP_IOU_THRESHOLD;
  params->pAnchors_0         = g_Anchors_0;
  params->pAnchors_1         = g_Anchors_1;
  params->boxe_0_scale       = NN_Info->outputs[0].scale.data[0];
  params->boxe_0_zero_point  = NN_Info->outputs[0].zeropoint.data[0];
  params->proba_0_scale      = NN_Info->outputs[1].scale.data[0];
  params->proba_0_zero_point = NN_Info->outputs[1].zeropoint.data[0];
  params->boxe_1_scale       = NN_Info->outputs[3].scale.data[0];
  params->boxe_1_zero_point  = NN_Info->outputs[3].zeropoint.data[0];
  params->proba_1_scale      = NN_Info->outputs[2].scale.data[0];
  params->proba_1_zero_point = NN_Info->outputs[2].zeropoint.data[0];
  for (int i = 0; i < AI_FD_BLAZEFACE_PP_TOTAL_BOXES; i++) {
    out_detections[i].pKeyPoints = &out_keyPoints[i * AI_FD_BLAZEFACE_PP_NB_KEYPOINTS];
  }
  error = fd_blazeface_pp_reset(params);
  return error;
}

int32_t app_postprocess_run(void *pInput[], int nb_input, void *pOutput, void *pInput_param)
{
  assert(nb_input == 4);
  int32_t error = AI_FD_PP_ERROR_NO;
  ((fd_blazeface_pp_static_param_t *) pInput_param)->nb_detect = 0;
  fd_pp_out_t *pObjDetOutput = (fd_pp_out_t *) pOutput;
  pObjDetOutput->pOutBuff = out_detections;
  fd_blazeface_pp_in_t pp_input = {
      .pRawDetections_0 = (int8_t *) pInput[0],
      .pScores_0        = (int8_t *) pInput[1],
      .pRawDetections_1 = (int8_t *) pInput[3],
      .pScores_1        = (int8_t *) pInput[2],
  };
  error = fd_blazeface_pp_process_int8(&pp_input, pObjDetOutput,
                                          (fd_blazeface_pp_static_param_t *) pInput_param);
  return error;
}
#endif
