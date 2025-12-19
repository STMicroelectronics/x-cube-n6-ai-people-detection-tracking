/*---------------------------------------------------------------------------------------------
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *--------------------------------------------------------------------------------------------*/

#include "fd_yunet_pp_if.h"
#include "vision_models_pp.h"


/* Offsets to access face detect blaze face input data */
#define AI_FD_YUNET_PP_XCENTER      (0)
#define AI_FD_YUNET_PP_YCENTER      (1)
#define AI_FD_YUNET_PP_WIDTHREL     (2)
#define AI_FD_YUNET_PP_HEIGHTREL    (3)

/* Can't be removed if qsort is not re-written... */
static int32_t AI_FD_PP_SORT_CLASS;


static int32_t fd_nms_comparator(const void *pa, const void *pb)
{
    fd_pp_outBuffer_t a = *(fd_pp_outBuffer_t *)pa;
    fd_pp_outBuffer_t b = *(fd_pp_outBuffer_t *)pb;

    float32_t diff = 0.0;
    float32_t a_weighted_conf = 0.0;
    float32_t b_weighted_conf = 0.0;

    if (a.class_index == AI_FD_PP_SORT_CLASS)
    {
        a_weighted_conf = a.conf;
    }
    else
    {
         a_weighted_conf = 0.0;
    }

    if (b.class_index == AI_FD_PP_SORT_CLASS)
    {
        b_weighted_conf = b.conf;
    }
    else
    {
         b_weighted_conf = 0.0;
    }

    diff = a_weighted_conf - b_weighted_conf;

    if (diff < 0) return 1;
    else if (diff > 0) return -1;
    return 0;
}

static int32_t fd_pp_nmsFiltering_centroid(fd_pp_out_t *pOutput,
                                           fd_yunet_pp_static_param_t *pInput_static_param)
{
    int32_t j, limit_counter, detections;
    limit_counter = 0;
    detections = 0;


    /* Counts the number of detections with class k */
    for (int32_t i = 0; i < pInput_static_param->nb_detect ; i ++)
    {
        detections++;
    }

    if (detections > 0)
    {
        /* Sorts detections based on class k */
        qsort(pOutput->pOutBuff,
              pInput_static_param->nb_detect,
              sizeof(fd_pp_outBuffer_t),
              fd_nms_comparator);

        for (int32_t i = 0; i < detections ; i ++)
        {
            if (pOutput->pOutBuff[i].conf == 0) continue;
            float32_t *a = &(pOutput->pOutBuff[i].x_center);
            for (j = i + 1; j < detections; j ++)
            {
                if (pOutput->pOutBuff[j].conf == 0) continue;
                float32_t *b = &(pOutput->pOutBuff[j].x_center);
                if (vision_models_box_iou(a, b) > pInput_static_param->iou_threshold)
                {
                    pOutput->pOutBuff[j].conf = 0;
                }
            }
        }

        /* Limits detections count */
        for (int32_t i = 0; i < detections; i++)
        {
            if ((limit_counter < pInput_static_param->max_boxes_limit) &&
                (pOutput->pOutBuff[i].conf != 0))
            {
                limit_counter++;
            }
            else
            {
                pOutput->pOutBuff[i].conf = 0;
            }
        }
    }
    return (AI_FD_PP_ERROR_NO);
}


static int32_t fd_pp_scoreFiltering_centroid(fd_pp_out_t *pOutput,
                                             fd_yunet_pp_static_param_t *pInput_static_param)
{
    int32_t det_count = 0;
    fd_pp_outBuffer_t *pOutBuff = (fd_pp_outBuffer_t *)pOutput->pOutBuff;

    for (int32_t i = 0; i < pInput_static_param->nb_detect; i++)
    {
        if (pOutBuff[i].conf >= pInput_static_param->conf_threshold)
        {
            pOutBuff[det_count].x_center    = pOutBuff[i].x_center;
            pOutBuff[det_count].y_center    = pOutBuff[i].y_center;
            pOutBuff[det_count].width       = pOutBuff[i].width;
            pOutBuff[det_count].height      = pOutBuff[i].height;
            pOutBuff[det_count].conf        = pOutBuff[i].conf;
            pOutBuff[det_count].class_index = pOutBuff[i].class_index;
            for (int32_t j = 0; j < pInput_static_param->nb_keypoints; j++)
            {
                pOutBuff[det_count].pKeyPoints[j].x    = pOutBuff[i].pKeyPoints[j].x;
                pOutBuff[det_count].pKeyPoints[j].y    = pOutBuff[i].pKeyPoints[j].y;
            }
            det_count++;
        }
    }
    pOutput->nb_detect = det_count;

    return (AI_FD_PP_ERROR_NO);
}

static int32_t fd_pp_level_decode_and_store_is8(int8_t *pRawBoxes,
                                                int8_t *pRawKps,
                                                int8_t *pCls,
                                                int8_t *pObj,
                                                fd_pp_out_t *pOutput,
                                                const int16_t *pAnchors,
                                                uint32_t inDetection,
                                                fd_yunet_pp_static_param_t *pInput_static_param,
                                                float32_t rawBoxes_scale,
                                                float32_t rawKps_scale,
                                                float32_t cls_scale,
                                                float32_t obj_scale,
                                                int8_t rawBoxes_zp,
                                                int8_t rawKps_zp,
                                                int8_t cls_zp,
                                                int8_t obj_zp)

{
  float32_t inv_size = 1.0f / pInput_static_param->in_size;
  int32_t kps_stride = pInput_static_param->nb_keypoints * 2;
  int32_t stride = pAnchors[1*2+0];

  int32_t det_count = pInput_static_param->nb_detect;
  fd_pp_outBuffer_t *pOutBuff = (fd_pp_outBuffer_t *)pOutput->pOutBuff;

  for (int32_t det = 0; det < (int32_t)inDetection; ++det)
  {
    float32_t  proba = (*pCls - cls_zp)*cls_scale * (*pObj - obj_zp)*obj_scale;
    if ( proba > pInput_static_param->conf_threshold) {
      if (det_count >= pInput_static_param->allocated_boxes) {
        return AI_FD_PP_ERROR_TRUNCATED;
      }
      /* read and activate objectness */
      pOutBuff[det_count].conf = proba;
      pOutBuff[det_count].class_index = 0;

      float32_t dequant;
      dequant = (float32_t)((int32_t)pRawBoxes[AI_FD_YUNET_PP_XCENTER] - rawBoxes_zp) * rawBoxes_scale;
      pOutBuff[det_count].x_center   = (dequant * (float32_t)stride + pAnchors[0]) * inv_size;

      dequant = (float32_t)((int32_t)pRawBoxes[AI_FD_YUNET_PP_YCENTER] - rawBoxes_zp) * rawBoxes_scale;
      pOutBuff[det_count].y_center   = (dequant * (float32_t)stride + pAnchors[1]) * inv_size;

      dequant = (float32_t)((int32_t)pRawBoxes[AI_FD_YUNET_PP_WIDTHREL] - rawBoxes_zp) * rawBoxes_scale;
      pOutBuff[det_count].width      = expf(dequant) * (float32_t)stride * inv_size;

      dequant = (float32_t)((int32_t)pRawBoxes[AI_FD_YUNET_PP_HEIGHTREL] - rawBoxes_zp) * rawBoxes_scale;
      pOutBuff[det_count].height     = expf(dequant) * (float32_t)stride * inv_size;

      for (uint32_t j = 0; j <  pInput_static_param->nb_keypoints; j++)
      {
          dequant = (float32_t)((int32_t)pRawKps[2*j + 0] - rawKps_zp) * rawKps_scale;
          pOutBuff[det_count].pKeyPoints[j].x = (dequant  * (float32_t)stride + pAnchors[0]) * inv_size;
          dequant = (float32_t)((int32_t)pRawKps[2*j + 1] - rawKps_zp) * rawKps_scale;
          pOutBuff[det_count].pKeyPoints[j].y = (dequant  * (float32_t)stride + pAnchors[1]) * inv_size;

      } // for anch

      det_count++;
    } // if threshold
    pCls++;
    pObj++;
    pRawBoxes+=4;
    pRawKps+=kps_stride;
    pAnchors+=2;

  } // for det
  pInput_static_param->nb_detect = det_count;

  return AI_FD_PP_ERROR_NO;
}

static int32_t fd_pp_level_decode_and_store(float32_t *pRawBoxes,
                                            float32_t *pRawKps,
                                            float32_t *pCls,
                                            float32_t *pObj,
                                            fd_pp_out_t *pOutput,
                                            const int16_t *pAnchors,
                                            uint32_t inDetection,
                                            fd_yunet_pp_static_param_t *pInput_static_param)

{
  float32_t inv_size = 1.0f / (float32_t)pInput_static_param->in_size;
  int32_t kps_stride = pInput_static_param->nb_keypoints * 2;
  int32_t stride = pAnchors[1*2+0];


  int32_t det_count = pInput_static_param->nb_detect;
  fd_pp_outBuffer_t *pOutBuff = (fd_pp_outBuffer_t *)pOutput->pOutBuff;

  for (int32_t det = 0; det < (int32_t)inDetection; ++det)
  {
    float32_t  proba = (*pCls) * (*pObj);
    if ( proba > pInput_static_param->conf_threshold) {
      if (det_count >= pInput_static_param->allocated_boxes) {
        return AI_FD_PP_ERROR_TRUNCATED;
      }
      /* read and activate objectness */
      pOutBuff[det_count].conf = proba;
      pOutBuff[det_count].class_index = 0;

      pOutBuff[det_count].x_center   = (pRawBoxes[AI_FD_YUNET_PP_XCENTER] * (float32_t)stride + (float32_t)pAnchors[0]) * inv_size;

      pOutBuff[det_count].y_center   = (pRawBoxes[AI_FD_YUNET_PP_YCENTER] * (float32_t)stride + (float32_t)pAnchors[1]) * inv_size;

      pOutBuff[det_count].width      = expf(pRawBoxes[AI_FD_YUNET_PP_WIDTHREL]) * (float32_t)stride * inv_size;

      pOutBuff[det_count].height     = expf(pRawBoxes[AI_FD_YUNET_PP_HEIGHTREL]) * (float32_t)stride * inv_size;
      for (uint32_t j = 0; j <  pInput_static_param->nb_keypoints; j++)
      {
          pOutBuff[det_count].pKeyPoints[j].x = (pRawKps[2*j + 0] * (float32_t)stride + (float32_t)pAnchors[0]) * inv_size;
          pOutBuff[det_count].pKeyPoints[j].y = (pRawKps[2*j + 1] * (float32_t)stride + (float32_t)pAnchors[1]) * inv_size;
      } // for anch

      det_count++;
    } // if threshold
    pCls++;
    pObj++;
    pRawBoxes+=4;
    pRawKps+=kps_stride;
    pAnchors+=2;

  } // for det
  pInput_static_param->nb_detect = det_count;

  return AI_FD_PP_ERROR_NO;

}

static int32_t fd_pp_getNNBoxes_centroid_is8(fd_yunet_pp_in_t *pInput,
                                             fd_pp_out_t *pOut,
                                             fd_yunet_pp_static_param_t *pInput_static_param)
{
  int32_t error   = AI_FD_PP_ERROR_NO;

  if (pOut->pOutBuff == NULL)
  {
    error   = AI_FD_PP_ERROR;
  }
  pInput_static_param->nb_detect = 0;

  if (error == AI_FD_PP_ERROR_NO)
  {
    int8_t *pBBoxRaw, *pKpsRaw, *pCls, *pObj;
    int16_t *pAnchors;
    uint32_t inDetections;
    float32_t bbxScale, kpsScale, clsScale, objScale;
    int8_t bbxZp, kpsZp, clsZp, objZp;

    //out 32
    bbxScale   = pInput_static_param->bbx_32_scale;
    bbxZp      = pInput_static_param->bbx_32_zero_point;
    kpsScale   = pInput_static_param->kps_32_scale;
    kpsZp      = pInput_static_param->kps_32_zero_point;
    clsScale   = pInput_static_param->cls_32_scale;
    clsZp      = pInput_static_param->cls_32_zero_point;
    objScale   = pInput_static_param->obj_32_scale;
    objZp      = pInput_static_param->obj_32_zero_point;
    inDetections = pInput_static_param->nb_detections_32;
    pBBoxRaw     = (int8_t *)pInput->pBBoxRaw_32;
    pKpsRaw      = (int8_t *)pInput->pKpsRaw_32;
    pCls         = (int8_t *)pInput->pCls_32;
    pObj         = (int8_t *)pInput->pObj_32;
    pAnchors     = (int16_t *)pInput_static_param->pAnchors_32;

    error = fd_pp_level_decode_and_store_is8(pBBoxRaw, pKpsRaw, pCls, pObj, pOut, pAnchors, inDetections, pInput_static_param,
                                     bbxScale, kpsScale, clsScale, objScale, bbxZp, kpsZp, clsZp, objZp);

    if (error == AI_FD_PP_ERROR_NO) {
      //out 16
      bbxScale   = pInput_static_param->bbx_16_scale;
      bbxZp      = pInput_static_param->bbx_16_zero_point;
      kpsScale   = pInput_static_param->kps_16_scale;
      kpsZp      = pInput_static_param->kps_16_zero_point;
      clsScale   = pInput_static_param->cls_16_scale;
      clsZp      = pInput_static_param->cls_16_zero_point;
      objScale   = pInput_static_param->obj_16_scale;
      objZp      = pInput_static_param->obj_16_zero_point;
      inDetections = pInput_static_param->nb_detections_16;
      pBBoxRaw     = (int8_t *)pInput->pBBoxRaw_16;
      pKpsRaw      = (int8_t *)pInput->pKpsRaw_16;
      pCls         = (int8_t *)pInput->pCls_16;
      pObj         = (int8_t *)pInput->pObj_16;
      pAnchors     = (int16_t *)pInput_static_param->pAnchors_16;

      error = fd_pp_level_decode_and_store_is8(pBBoxRaw, pKpsRaw, pCls, pObj, pOut, pAnchors, inDetections, pInput_static_param,
                                       bbxScale, kpsScale, clsScale, objScale, bbxZp, kpsZp, clsZp, objZp);
    }
    if (error == AI_FD_PP_ERROR_NO) {
      //out 8
      bbxScale   = pInput_static_param->bbx_8_scale;
      bbxZp      = pInput_static_param->bbx_8_zero_point;
      kpsScale   = pInput_static_param->kps_8_scale;
      kpsZp      = pInput_static_param->kps_8_zero_point;
      clsScale   = pInput_static_param->cls_8_scale;
      clsZp      = pInput_static_param->cls_8_zero_point;
      objScale   = pInput_static_param->obj_8_scale;
      objZp      = pInput_static_param->obj_8_zero_point;
      inDetections = pInput_static_param->nb_detections_8;
      pBBoxRaw     = (int8_t *)pInput->pBBoxRaw_8;
      pKpsRaw      = (int8_t *)pInput->pKpsRaw_8;
      pCls         = (int8_t *)pInput->pCls_8;
      pObj         = (int8_t *)pInput->pObj_8;
      pAnchors     = (int16_t *)pInput_static_param->pAnchors_8;

      error = fd_pp_level_decode_and_store_is8(pBBoxRaw, pKpsRaw, pCls, pObj, pOut, pAnchors, inDetections, pInput_static_param,
                                       bbxScale, kpsScale, clsScale, objScale, bbxZp, kpsZp, clsZp, objZp);
    }
  }
  return (error);
}

static int32_t fd_pp_getNNBoxes_centroid(fd_yunet_pp_in_t *pInput,
                                         fd_pp_out_t *pOut,
                                         fd_yunet_pp_static_param_t *pInput_static_param)
{
  int32_t error   = AI_FD_PP_ERROR_NO;

  if (pOut->pOutBuff == NULL)
  {
    error   = AI_FD_PP_ERROR;
  }
  pInput_static_param->nb_detect = 0;

  if (error == AI_FD_PP_ERROR_NO)
  {
    float32_t *pBBoxRaw, *pKpsRaw, *pCls, *pObj;
    int16_t *pAnchors;
    uint32_t inDetections;

    //out 32
    inDetections = pInput_static_param->nb_detections_32;
    pBBoxRaw     = (float32_t *)pInput->pBBoxRaw_32;
    pKpsRaw      = (float32_t *)pInput->pKpsRaw_32;
    pCls         = (float32_t *)pInput->pCls_32;
    pObj         = (float32_t *)pInput->pObj_32;
    pAnchors     = (int16_t *)pInput_static_param->pAnchors_32;

    error = fd_pp_level_decode_and_store(pBBoxRaw, pKpsRaw, pCls, pObj, pOut, pAnchors, inDetections, pInput_static_param);

    if (error == AI_FD_PP_ERROR_NO) {
      //out 16
      inDetections = pInput_static_param->nb_detections_16;
      pBBoxRaw     = (float32_t *)pInput->pBBoxRaw_16;
      pKpsRaw      = (float32_t *)pInput->pKpsRaw_16;
      pCls         = (float32_t *)pInput->pCls_16;
      pObj         = (float32_t *)pInput->pObj_16;
      pAnchors     = (int16_t *)pInput_static_param->pAnchors_16;
      
      error = fd_pp_level_decode_and_store(pBBoxRaw, pKpsRaw, pCls, pObj, pOut, pAnchors, inDetections, pInput_static_param);
    }

    if (error == AI_FD_PP_ERROR_NO) {
      //out 8
      inDetections = pInput_static_param->nb_detections_8;
      pBBoxRaw     = (float32_t *)pInput->pBBoxRaw_8;
      pKpsRaw      = (float32_t *)pInput->pKpsRaw_8;
      pCls         = (float32_t *)pInput->pCls_8;
      pObj         = (float32_t *)pInput->pObj_8;
      pAnchors     = (int16_t *)pInput_static_param->pAnchors_8;

      error = fd_pp_level_decode_and_store(pBBoxRaw, pKpsRaw, pCls, pObj, pOut, pAnchors, inDetections, pInput_static_param);
    }
  }
  return (error);
}

/* ----------------------       Exported routines      ---------------------- */

int32_t fd_yunet_pp_reset(fd_yunet_pp_static_param_t *pInput_static_param)
{
  if (pInput_static_param == NULL)
  {
    return (AI_FD_PP_ERROR);
  }
  return (AI_FD_PP_ERROR_NO);
}

int32_t fd_yunet_pp_process_int8(fd_yunet_pp_in_t *pInput,
                              fd_pp_out_t *pOutput,
                              fd_yunet_pp_static_param_t *pInput_static_param)
{
  int32_t error   = AI_FD_PP_ERROR_NO;

  if (pInput == NULL)
  {
    return (AI_FD_PP_ERROR);
  }
  if (pInput == NULL)
  {
    return (AI_FD_PP_ERROR);
  }
  if (pInput_static_param == NULL)
  {
    return (AI_FD_PP_ERROR);
  }

  /* Call Get NN boxes first */
  error = fd_pp_getNNBoxes_centroid_is8(pInput,
                                        pOutput,
                                        pInput_static_param);

  if (error != AI_FD_PP_ERROR_NO) return (error);

  /* Then NMS */
  error = fd_pp_nmsFiltering_centroid(pOutput,
                                      pInput_static_param);

  if (error != AI_FD_PP_ERROR_NO) return (error);

  /* And score re-filtering */
  error = fd_pp_scoreFiltering_centroid(pOutput,
                                        pInput_static_param);

  return (error);
}

int32_t fd_yunet_pp_process(fd_yunet_pp_in_t *pInput,
                            fd_pp_out_t *pOutput,
                            fd_yunet_pp_static_param_t *pInput_static_param)
{
  int32_t error   = AI_FD_PP_ERROR_NO;

  if (pInput == NULL)
  {
    return (AI_FD_PP_ERROR);
  }
  if (pInput == NULL)
  {
    return (AI_FD_PP_ERROR);
  }
  if (pInput_static_param == NULL)
  {
    return (AI_FD_PP_ERROR);
  }

  /* Call Get NN boxes first */
  error = fd_pp_getNNBoxes_centroid(pInput,
                                    pOutput,
                                    pInput_static_param);

  if (error != AI_FD_PP_ERROR_NO) return (error);

  /* Then NMS */
  error = fd_pp_nmsFiltering_centroid(pOutput,
                                      pInput_static_param);

  if (error != AI_FD_PP_ERROR_NO) return (error);

  /* And score re-filtering */
  error = fd_pp_scoreFiltering_centroid(pOutput,
                                        pInput_static_param);

  return (error);
}
