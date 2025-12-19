/**
 ******************************************************************************
 * @file    usbx.h
 * @author  AIS Application Team
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef _USBX_H_
#define _USBX_H_

#include "usbx_conf.h"
#include "usb_desc.h"

int usbx_init(PCD_HandleTypeDef *pcd_handle, PCD_TypeDef *pcd_instance, uvc_ctx_t *p_ctx);
uint32_t usbx_read(uint8_t* payload);
void usbx_write(unsigned char *msg, uint32_t len);

#endif
