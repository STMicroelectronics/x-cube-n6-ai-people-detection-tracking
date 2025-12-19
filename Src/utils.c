 /**
 ******************************************************************************
 * @file    utils.c
 * @author  GPM Application Team
 *
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

#include "utils.h"

#include "stai_network.h"

void Run_Inference(stai_network *network_instance)
{
  stai_return_code ret;

  do {
    ret = stai_network_run(network_instance, STAI_MODE_ASYNC);
    if (ret == STAI_RUNNING_WFE)
      LL_ATON_OSAL_WFE();
  } while (ret == STAI_RUNNING_WFE || ret == STAI_RUNNING_NO_WFE);

  ret = stai_ext_network_new_inference(network_instance);
  assert(ret == STAI_SUCCESS);
}
