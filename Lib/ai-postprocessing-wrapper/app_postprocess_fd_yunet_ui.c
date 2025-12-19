 /**
 ******************************************************************************
 * @file    app_postprocess_fd_yunet_ui.c
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

#if POSTPROCESS_TYPE == POSTPROCESS_FD_YUNET_UI

#include "fd_yunet_pp_if.h"
#include "fd_yunet_anchors_32.h"
#include "fd_yunet_anchors_16.h"
#include "fd_yunet_anchors_8.h"

#define AI_FD_YUNET_PP_MAX_BOXES MAX(AI_FD_YUNET_PP_MAX_BOXES_LIMIT, AI_FD_YUNET_PP_OUT_32_NB_BOXES + AI_FD_YUNET_PP_OUT_16_NB_BOXES + AI_FD_YUNET_PP_OUT_8_NB_BOXES)
static fd_pp_outBuffer_t out_detections[AI_FD_YUNET_PP_MAX_BOXES];
static fd_pp_keyPoints_t out_keyPoints[AI_FD_YUNET_PP_MAX_BOXES*2*AI_FD_YUNET_PP_NB_KEYPOINTS];

int32_t app_postprocess_init(void *params_postprocess, stai_network_info *NN_Info)
{
	int32_t error = AI_FD_PP_ERROR_NO;
	fd_yunet_pp_static_param_t *params = (fd_yunet_pp_static_param_t *) params_postprocess;

	params->in_size            = AI_FD_YUNET_PP_IMG_SIZE;
	params->nb_keypoints       = AI_FD_YUNET_PP_NB_KEYPOINTS;
	params->nb_detections_32   = AI_FD_YUNET_PP_OUT_32_NB_BOXES;
	params->nb_detections_16   = AI_FD_YUNET_PP_OUT_16_NB_BOXES;
	params->nb_detections_8    = AI_FD_YUNET_PP_OUT_8_NB_BOXES;
	params->max_boxes_limit    = AI_FD_YUNET_PP_MAX_BOXES_LIMIT;
	params->allocated_boxes    = AI_FD_YUNET_PP_MAX_BOXES;
	params->conf_threshold     = AI_FD_YUNET_PP_CONF_THRESHOLD;
	params->iou_threshold      = AI_FD_YUNET_PP_IOU_THRESHOLD;
	params->pAnchors_32        = g_Anchors_32;
	params->pAnchors_16        = g_Anchors_16;
	params->pAnchors_8         = g_Anchors_8;

	/* Output ordering is assumed per stride: cls, obj, bbx, kps */
	/* Stride 32 */
	params->cls_32_scale       = NN_Info->outputs[2].scale.data[0];
	params->cls_32_zero_point  = NN_Info->outputs[2].zeropoint.data[0];
	params->obj_32_scale       = NN_Info->outputs[5].scale.data[0];
	params->obj_32_zero_point  = NN_Info->outputs[5].zeropoint.data[0];
	params->bbx_32_scale       = NN_Info->outputs[8].scale.data[0];
	params->bbx_32_zero_point  = NN_Info->outputs[8].zeropoint.data[0];
	params->kps_32_scale       = NN_Info->outputs[11].scale.data[0];
	params->kps_32_zero_point  = NN_Info->outputs[11].zeropoint.data[0];
	/* Stride 16 */
	params->cls_16_scale       = NN_Info->outputs[1].scale.data[0];
	params->cls_16_zero_point  = NN_Info->outputs[1].zeropoint.data[0];
	params->obj_16_scale       = NN_Info->outputs[4].scale.data[0];
	params->obj_16_zero_point  = NN_Info->outputs[4].zeropoint.data[0];
	params->bbx_16_scale       = NN_Info->outputs[7].scale.data[0];
	params->bbx_16_zero_point  = NN_Info->outputs[7].zeropoint.data[0];
	params->kps_16_scale       = NN_Info->outputs[10].scale.data[0];
	params->kps_16_zero_point  = NN_Info->outputs[10].zeropoint.data[0];
	/* Stride 8 */
	params->cls_8_scale        = NN_Info->outputs[0].scale.data[0];
	params->cls_8_zero_point   = NN_Info->outputs[0].zeropoint.data[0];
	params->obj_8_scale        = NN_Info->outputs[3].scale.data[0];
	params->obj_8_zero_point   = NN_Info->outputs[3].zeropoint.data[0];
	params->bbx_8_scale        = NN_Info->outputs[6].scale.data[0];
	params->bbx_8_zero_point   = NN_Info->outputs[6].zeropoint.data[0];
	params->kps_8_scale        = NN_Info->outputs[9].scale.data[0];
	params->kps_8_zero_point   = NN_Info->outputs[9].zeropoint.data[0];

	for (int i = 0; i < AI_FD_YUNET_PP_MAX_BOXES; i++) {
		out_detections[i].pKeyPoints = &out_keyPoints[i*2*AI_FD_YUNET_PP_NB_KEYPOINTS];
	}
	error = fd_yunet_pp_reset(params);
	return error;
}

int32_t app_postprocess_run(void *pInput[], int nb_input, void *pOutput, void *pInput_param)
{
	assert(nb_input == 12);
	int32_t error = AI_FD_PP_ERROR_NO;
	((fd_yunet_pp_static_param_t *) pInput_param)->nb_detect = 0;
	fd_pp_out_t *pFdOutput = (fd_pp_out_t *) pOutput;
	pFdOutput->pOutBuff = out_detections;

	fd_yunet_pp_in_t pp_input = {
			.pCls_32      = (int8_t *) pInput[2],
			.pObj_32      = (int8_t *) pInput[5],
			.pBBoxRaw_32  = (int8_t *) pInput[8],
			.pKpsRaw_32   = (int8_t *) pInput[11],
			.pCls_16      = (int8_t *) pInput[1],
			.pObj_16      = (int8_t *) pInput[4],
			.pBBoxRaw_16  = (int8_t *) pInput[7],
			.pKpsRaw_16   = (int8_t *) pInput[10],
			.pCls_8       = (int8_t *) pInput[0],
			.pObj_8       = (int8_t *) pInput[3],
			.pBBoxRaw_8   = (int8_t *) pInput[6],
			.pKpsRaw_8    = (int8_t *) pInput[9],
	};

	error = fd_yunet_pp_process_int8(&pp_input, pFdOutput,
																	 (fd_yunet_pp_static_param_t *) pInput_param);
	return error;
}

#endif
