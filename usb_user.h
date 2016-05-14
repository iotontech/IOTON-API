/* Ioton USBCDC Library
 * Copyright (c) 2016 Ioton Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __USB_USER_H
#define __USB_USER_H


/* Includes ------------------------------------------------------------------*/
#include <sys/stat.h>
#include <stdio.h>
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if_template.h"
#include "usbd_desc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"


#ifdef __cplusplus
extern "C"{
#endif

void initUSB (void);

#ifdef __cplusplus
} // extern "C"
#endif


#endif /* __USB_USER_H */
