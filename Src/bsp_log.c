/**
******************************************************************************
  * File Name          : bsp_log.c
  * Description        : 
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  /* Includes ------------------------------------------------------------------*/
  #include "bsp_log.h"
  #include "bsp_protocol.h"
    
 static uint8_t DoorBoardLogInfo[24];

 static uint8_t getXORCode(uint8_t* pData,uint16_t len)
{
  uint8_t ret;
  uint16_t i;
  ret = pData[0];
  for(i = 1; i < len; i++)
  {
    ret ^=pData[i];
  }
  return ret;
}

 void BSP_UpDeviceID(uint32_t deviceID)
 {
      DoorBoardLogInfo[0] = (uint8_t)(deviceID >> 24);
      DoorBoardLogInfo[1] = (uint8_t)(deviceID >> 16);
      DoorBoardLogInfo[2] = (uint8_t)(deviceID >> 8);
      DoorBoardLogInfo[3] = (uint8_t)(deviceID);
 }
 void BSP_UpCurrentPeak(uint16_t tCurrentPeakValue)
 {
      DoorBoardLogInfo[4] = (uint8_t)(tCurrentPeakValue >> 8);
      DoorBoardLogInfo[5] = (uint8_t)tCurrentPeakValue;
 }
 
 void BSP_UpAtmosphereStatus(uint8_t tMode)
 {
      DoorBoardLogInfo[6] = tMode;
 }
 void BSP_UpDoorOpenSpeed(uint8_t tSpeed)
 {
      DoorBoardLogInfo[7] = tSpeed;
 }
 void BSP_UpMotorSpeed(uint16_t tMotorSpeed)
 {
      DoorBoardLogInfo[8] = (uint8_t)(tMotorSpeed >> 8);
      DoorBoardLogInfo[9] = (uint8_t) tMotorSpeed;
 }
 
 void BSP_UpMotorStatus(uint8_t tMotorStatus)
 {
      DoorBoardLogInfo[10] = tMotorStatus;
 }
 void BSP_UpDoorOpenMode(uint8_t tOpenMode)
 {
      DoorBoardLogInfo[11] = tOpenMode;
 }
 void BSP_UpGentleStatus(uint8_t tStatus)
 {
      DoorBoardLogInfo[12] = tStatus;
 }
 void BSP_UpPressureWaveCondition(uint8_t tStatus)
 {
      DoorBoardLogInfo[13] = tStatus;
 }
 
 void BSP_UpRadarParams(uint8_t* pData)
 {
      DoorBoardLogInfo[14] = (*pData);
      DoorBoardLogInfo[15] = *(pData + 1);
      DoorBoardLogInfo[16] = *(pData + 2);
      DoorBoardLogInfo[17] = *(pData + 3);
      DoorBoardLogInfo[18] = *(pData + 4);
 }
 
 void BSP_UpRadarStatus(uint8_t* pData)
 {
      DoorBoardLogInfo[19] = (*pData);
      DoorBoardLogInfo[20] = *(pData + 1);
      DoorBoardLogInfo[21] = *(pData + 2);
      DoorBoardLogInfo[22] = *(pData + 3);
      DoorBoardLogInfo[23] = *(pData + 4);   
 }
 
 BSP_StatusTypeDef BSP_ReportLogInfo(void)
 {
    BSP_StatusTypeDef status = BSP_OK;
    uint8_t i;
    uint8_t tTempbuffer[31];
    tTempbuffer[0] = 0x5B;
    tTempbuffer[1] = 0xD2;
    tTempbuffer[2] = 0x01;
    tTempbuffer[3] = 0x00;
    tTempbuffer[4] = 24;
    tTempbuffer[30] = 0x5D;
    
    for(i = 0; i < 24; i++)
    {
      tTempbuffer[5 + i] = DoorBoardLogInfo[i];
    }
    
    tTempbuffer[29] = getXORCode((tTempbuffer + 1), 28);
    
    status = BSP_SendDataToDriverBoard(tTempbuffer,31,0xFFFF);
    
    return status;
 }
  
  /**
  * @}
  */
  /**
  * @}
  */
  /*****************************END OF FILE**************************************/
