 /**
 ******************************************************************************
 * @file    app_postprocess_od_blazeface_ui.c
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

#if POSTPROCESS_TYPE == POSTPROCESS_OD_BLAZEFACE_UI
#include "fd_blazeface_anchors_0.h"
#include "fd_blazeface_anchors_1.h"
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif
static od_pp_outBuffer_t out_detections[MAX(AI_OD_BLAZEFACE_PP_MAX_BOXES_LIMIT, AI_OD_BLAZEFACE_PP_OUT_0_NB_BOXES + AI_OD_BLAZEFACE_PP_OUT_1_NB_BOXES)];

int32_t app_postprocess_init(void *params_postprocess, stai_network_info *NN_Info)
{
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
  od_blazeface_pp_static_param_t *params = (od_blazeface_pp_static_param_t *) params_postprocess;

  params->in_size            = AI_OD_BLAZEFACE_PP_IMG_SIZE;
  params->nb_classes         = AI_OD_BLAZEFACE_PP_NB_CLASSES;
  params->nb_keypoints       = AI_OD_BLAZEFACE_PP_NB_KEYPOINTS;
  params->nb_detections_0    = AI_OD_BLAZEFACE_PP_OUT_0_NB_BOXES;
  params->nb_detections_1    = AI_OD_BLAZEFACE_PP_OUT_1_NB_BOXES;
  params->max_boxes_limit    = AI_OD_BLAZEFACE_PP_MAX_BOXES_LIMIT;
  params->conf_threshold     = AI_OD_BLAZEFACE_PP_CONF_THRESHOLD;
  params->iou_threshold      = AI_OD_BLAZEFACE_PP_IOU_THRESHOLD;
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
  error = od_blazeface_pp_reset(params);
  return error;
}

int32_t app_postprocess_run(void *pInput[], int nb_input, void *pOutput, void *pInput_param)
{
  assert(nb_input == 4);
  int32_t error = AI_OD_POSTPROCESS_ERROR_NO;
  ((od_blazeface_pp_static_param_t *) pInput_param)->nb_detect = 0;
  od_pp_out_t *pObjDetOutput = (od_pp_out_t *) pOutput;
  pObjDetOutput->pOutBuff = out_detections;
  od_blazeface_pp_in_t pp_input = {
      .pRawDetections_0 = (int8_t *) pInput[0],
      .pScores_0        = (int8_t *) pInput[1],
      .pRawDetections_1 = (int8_t *) pInput[3],
      .pScores_1        = (int8_t *) pInput[2],
  };
  error = od_blazeface_pp_process_int8(&pp_input, pObjDetOutput,
                                          (od_blazeface_pp_static_param_t *) pInput_param);
  return error;
}
#endif
