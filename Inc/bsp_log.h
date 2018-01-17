/**
  ******************************************************************************
  * File Name          : bsp_log.h
  * Description        : about log informations
  *
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __bsp_log_H
#define __bsp_log_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_common.h"
#include "BSP_DAC5571.h"

 void BSP_UpDeviceID(uint32_t deviceID);
 void BSP_UpCurrentPeak(uint16_t tCurrentPeakValue);
 void BSP_UpAtmosphereStatus(uint8_t tMode);
 void BSP_UpDoorOpenSpeed(uint8_t tSpeed);
 void BSP_UpMotorSpeed(uint16_t tMotorSpeed);
 void BSP_UpMotorStatus(uint8_t tMotorStatus);
 void BSP_UpDoorOpenMode(uint8_t tOpenMode);
 void BSP_UpGentleStatus(uint8_t tStatus);
 void BSP_UpPressureWaveCondition(uint8_t tStatus);
 void BSP_UpRadarParams(uint8_t* pData);
 void BSP_UpRadarStatus(uint8_t* pData);
   
 BSP_StatusTypeDef BSP_ReportLogInfo(void);
 
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__bsp_log_H */