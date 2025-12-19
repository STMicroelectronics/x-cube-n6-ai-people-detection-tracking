 /**
 ******************************************************************************
 * @file    app.h
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

#ifndef APP_H
#define APP_H

#include "stai_network.h"

#define NN_WIDTH STAI_NETWORK_IN_1_WIDTH
#define NN_HEIGHT STAI_NETWORK_IN_1_HEIGHT

void app_run(void);

#endif