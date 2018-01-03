/**
******************************************************************************
  * File Name          : bsp_protocol.c
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
#include "bsp_protocol.h"
#include "bsp_motor.h"

uint8_t AckCmdBuffer[6];
uint8_t DriverBoardCmdBuffer[BSP_CMD_LEN + BSP_DATA_LEN];

USARTRECIVETYPE     DriverBoardUsartType;

extern PROTOCOLCMD  gDriverBoardProtocolCmd;
extern MOTORMACHINE gMotorMachine;

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

/*******************************************************************************
*
*       Function        :BSP_SendRequestCmd()
*
*       Input           :pPROTOCOLCMD pRequestCmd,uint8_t *pCmdDataBuffer
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/26
*       Author          :bertz
*******************************************************************************/
static BSP_StatusTypeDef BSP_SendRequestCmd(pPROTOCOLCMD pRequestCmd,uint8_t *pCmdDataBuffer)
{
  uint16_t dataLength = 0;
  uint8_t tempXOR    = 0;
  BSP_StatusTypeDef state = BSP_OK;
  if(0 == pRequestCmd->HandingFlag)
  {
    pRequestCmd->HandingFlag = 1;
    *(pCmdDataBuffer + 1) = pRequestCmd->CmdType;
    *(pCmdDataBuffer + 2)= pRequestCmd->CmdParam;
    *(pCmdDataBuffer + 3)= pRequestCmd->DataLengthHight;
    *(pCmdDataBuffer + 4)= pRequestCmd->DataLengthLow;
    dataLength = pRequestCmd->DataLength;
    *(pCmdDataBuffer + REQUESTFIXEDCOMMANDLEN + dataLength - 1) = 0x5D;
    pRequestCmd->TotalLength = dataLength + REQUESTFIXEDCOMMANDLEN;
    /* Calculate XOR */
    tempXOR = getXORCode(pCmdDataBuffer + 1, pRequestCmd->TotalLength - 3);
    
    *(pCmdDataBuffer + dataLength + REQUESTFIXEDCOMMANDLEN - 2) = (uint8_t)tempXOR;
    
    pRequestCmd->RevOrSendFlag = 1;
    pRequestCmd->RevEchoFlag = 0;
  }
  return state;   
}
/*******************************************************************************
*
*       Function        :BSP_CoreBoardUsartReceive_IDLE()
*
*       Input           :UART_HandleTypeDef
*
*       Return          :void
*
*       Description     :--
*
*
*       Data            :2017/12/25
*       Author          :bertz
*******************************************************************************/
void BSP_DriverBoardUsartReceive_IDLE(UART_HandleTypeDef *huart)
{
  uint32_t  temp;
  if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_DMAStop(&huart1);
    temp = huart1.hdmarx->Instance->CNDTR;
    DriverBoardUsartType.RX_Size = BSP_RX_LEN - temp;
    DriverBoardUsartType.RX_Flag = 1;
    HAL_UART_Receive_DMA(&huart1,DriverBoardUsartType.RX_pData,BSP_RX_LEN);
  }
}
  
/*******************************************************************************
*
*       Function        :BSP_DriverBoardProtocolInit()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/27
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_DriverBoardProtocolInit(void)
{
  BSP_StatusTypeDef state = BSP_OK;
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1,DriverBoardUsartType.RX_pData,BSP_RX_LEN);
  HAL_GPIO_WritePin(CTR485_EN1_GPIO_Port,CTR485_EN1_Pin,GPIO_PIN_RESET);
  
  DriverBoardCmdBuffer[0] = 0x5B;
  AckCmdBuffer[0] = 0x5B;
  AckCmdBuffer[5] = 0x5D;
  
  
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_SendDataToDriverBoard()
*
*       Input           :uint8_t *pData, uint16_t size,uint32_t Timeout
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/27
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_SendDataToDriverBoard(uint8_t *pData, uint16_t size,uint32_t Timeout)
{
  BSP_StatusTypeDef state = BSP_OK;
  HAL_GPIO_WritePin(CTR485_EN1_GPIO_Port,CTR485_EN1_Pin,GPIO_PIN_SET);
  state = (BSP_StatusTypeDef)HAL_UART_Transmit(&huart1,pData,size,Timeout);
  HAL_GPIO_WritePin(CTR485_EN1_GPIO_Port,CTR485_EN1_Pin,GPIO_PIN_RESET);
  if(BSP_OK != state)
  {
    state = BSP_ERROR;
  }
  return state;
}
/*******************************************************************************
*
*       Function        :BSP_SendRequestCmdToDriverBoard()
*
*       Input           :pPROTOCOLCMD pRequestCmd
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/27
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_SendRequestCmdToDriverBoard(pPROTOCOLCMD pRequestCmd)
{
  BSP_StatusTypeDef state = BSP_OK;
  if(pRequestCmd->RevRequestFlag)
  {
    return state;
  }
  state = BSP_SendRequestCmd(pRequestCmd, DriverBoardCmdBuffer);
  state = BSP_SendDataToDriverBoard(DriverBoardCmdBuffer, pRequestCmd->TotalLength,0xFFFF);
  
  return state;
}

BSP_StatusTypeDef       BSP_AckRequestCmdFromDriverBoard(pPROTOCOLCMD pRequestCmd);
/*******************************************************************************
*
*       Function        :BSP_AckRequestCmdFromDriverBoard()
*
*       Input           :pPROTOCOLCMD pRequestCmd
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_AckRequestCmdFromDriverBoard(pPROTOCOLCMD pRequestCmd)
{
  BSP_StatusTypeDef state = BSP_OK;
  AckCmdBuffer[1] = pRequestCmd->AckCmdCode;
  AckCmdBuffer[2] = pRequestCmd->AckCodeH;
  AckCmdBuffer[3] = pRequestCmd->AckCodeL;
  AckCmdBuffer[4] = getXORCode(AckCmdBuffer + 1, 3);
  
  state = BSP_SendDataToDriverBoard(AckCmdBuffer,6,0xFFFF); 
  return state;
}


/*******************************************************************************
*
*       Function        :BSP_HandingUartDataFromDriverBoard()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/27
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_HandingUartDataFromDriverBoard(void)
{
  BSP_StatusTypeDef    state = BSP_OK;
  uint8_t xorTemp;
  uint16_t i;
  
  if(!DriverBoardUsartType.RX_Flag)
  {
    return state;
  }
  
  DriverBoardUsartType.RX_Flag = 0;
  if(gDriverBoardProtocolCmd.RevOrSendFlag)
  {
    return state;
  }
  
  if(gDriverBoardProtocolCmd.RevRequestFlag)
  {
    return state;
  }
  
  if(gDriverBoardProtocolCmd.RevDataCnt < gDriverBoardProtocolCmd.DataLength && 0 != gDriverBoardProtocolCmd.RevDataCnt)
  {
    for(i = 0; i < DriverBoardUsartType.RX_Size; i++)
    {
      DriverBoardCmdBuffer[5 + gDriverBoardProtocolCmd.RevDataCnt + i] = *(DriverBoardUsartType.RX_pData + i);
      gDriverBoardProtocolCmd.RevDataCnt ++;
      if(gDriverBoardProtocolCmd.DataLength == gDriverBoardProtocolCmd.RevDataCnt)
      {
        gDriverBoardProtocolCmd.XOR8BIT = *(DriverBoardUsartType.RX_pData + i + 1);
        
        if(0x5D != *(DriverBoardUsartType.RX_pData + i + 2))
        {
          gDriverBoardProtocolCmd.RevDataCnt  = 0;
          gDriverBoardProtocolCmd.DataLength  = 0;
          gDriverBoardProtocolCmd.TotalLength = 0;
          return state;
        }
        gDriverBoardProtocolCmd.TotalLength  = gDriverBoardProtocolCmd.DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to check XOR code */
        xorTemp = getXORCode(DriverBoardCmdBuffer + 1, gDriverBoardProtocolCmd.TotalLength - 3);
        if(gDriverBoardProtocolCmd.XOR8BIT != xorTemp)
        {
          gDriverBoardProtocolCmd.TotalLength = 0;
          return state;
        }
        gDriverBoardProtocolCmd.RevRequestFlag = 1;
        return state;
      }
    }
    return state;
  }
  
  if(0 == gDriverBoardProtocolCmd.TotalLength)
  {
    if(0x5B != *(DriverBoardUsartType.RX_pData))
    {
      return state;
    }
    
    if(0xA0 == (*(DriverBoardUsartType.RX_pData + 1) & 0xF0) && 0x5D == *(DriverBoardUsartType.RX_pData + ACKFIXEDCOMMANDLEN - 1 ))
    {
      gDriverBoardProtocolCmd.AckCmdCode        =*(DriverBoardUsartType.RX_pData + 1);
      gDriverBoardProtocolCmd.AckCodeH          =*(DriverBoardUsartType.RX_pData + 2);
      gDriverBoardProtocolCmd.AckCodeL          =*(DriverBoardUsartType.RX_pData + 3);
      gDriverBoardProtocolCmd.AckXOR8BIT        =*(DriverBoardUsartType.RX_pData + 4);
      
      /* Here to add XOR code */
      xorTemp = getXORCode(DriverBoardUsartType.RX_pData + 1, 3);
      if(gDriverBoardProtocolCmd.AckXOR8BIT != xorTemp)
      {
        return state;
      }
      gDriverBoardProtocolCmd.RevEchoFlag = 1;
      return state;
    }
    gDriverBoardProtocolCmd.CmdType             =*(DriverBoardUsartType.RX_pData + 1);
    gDriverBoardProtocolCmd.CmdParam            =*(DriverBoardUsartType.RX_pData + 2);
    gDriverBoardProtocolCmd.DataLengthHight     =*(DriverBoardUsartType.RX_pData + 3);
    gDriverBoardProtocolCmd.DataLengthLow       =*(DriverBoardUsartType.RX_pData + 4);
    
    DriverBoardCmdBuffer[1]     = gDriverBoardProtocolCmd.CmdType;
    DriverBoardCmdBuffer[2]     = gDriverBoardProtocolCmd.CmdParam;
    DriverBoardCmdBuffer[3]     = gDriverBoardProtocolCmd.DataLengthHight;
    DriverBoardCmdBuffer[4]     = gDriverBoardProtocolCmd.DataLengthLow;
    
    gDriverBoardProtocolCmd.DataLength = (gDriverBoardProtocolCmd.DataLengthHight << 8) + gDriverBoardProtocolCmd.DataLengthLow;
    
    if(0 == gDriverBoardProtocolCmd.DataLength)
    {
      if(0x5D != *(DriverBoardUsartType.RX_pData + REQUESTFIXEDCOMMANDLEN - 1))
      {
        return state;
      }
      
      
      
      gDriverBoardProtocolCmd.XOR8BIT           = *(DriverBoardUsartType.RX_pData + 5);
      gDriverBoardProtocolCmd.TotalLength       = REQUESTFIXEDCOMMANDLEN;
      /* here to check XOR */
      xorTemp = getXORCode(DriverBoardCmdBuffer + 1, gDriverBoardProtocolCmd.TotalLength - 3);
      if(gDriverBoardProtocolCmd.XOR8BIT != xorTemp)
      {
        gDriverBoardProtocolCmd.TotalLength       = 0;
        return state;
      }
      gDriverBoardProtocolCmd.RevRequestFlag    = 1;
      
      return state;
    }
    
    for(i = 5; i < DriverBoardUsartType.RX_Size; i++)
    {
      DriverBoardCmdBuffer[i] = *(DriverBoardUsartType.RX_pData + i);
      gDriverBoardProtocolCmd.RevDataCnt ++;
      if(gDriverBoardProtocolCmd.DataLength == gDriverBoardProtocolCmd.RevDataCnt)
      {
        if(0x5D != *(DriverBoardUsartType.RX_pData + REQUESTFIXEDCOMMANDLEN + gDriverBoardProtocolCmd.RevDataCnt - 1))
        {
          gDriverBoardProtocolCmd.RevDataCnt            = 0;
          gDriverBoardProtocolCmd.DataLength            = 0;
          gDriverBoardProtocolCmd.TotalLength           = 0;
          return state;
        }
        gDriverBoardProtocolCmd.XOR8BIT = *(DriverBoardUsartType.RX_pData + i + 1);
        gDriverBoardProtocolCmd.TotalLength     = REQUESTFIXEDCOMMANDLEN + gDriverBoardProtocolCmd.DataLength;
        /* here add to check XOR */
        xorTemp = getXORCode(DriverBoardCmdBuffer + 1, gDriverBoardProtocolCmd.TotalLength - 3);
        if(gDriverBoardProtocolCmd.XOR8BIT != xorTemp)
        {
          gDriverBoardProtocolCmd.TotalLength     = 0;
          return state;
        }
        
        gDriverBoardProtocolCmd.RevRequestFlag  = 1; 
        return state;
      }
    } 
  }
  return state;
}


/*******************************************************************************
*
*       Function        :BSP_HandingCmdFromDriverBoard()
*
*       Input           :pPROTOCOLCMD pRequestCmd
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_HandingCmdFromDriverBoard(pPROTOCOLCMD pRequestCmd)
{
  BSP_StatusTypeDef state = BSP_OK;
  
  if(1 == pRequestCmd->RevRequestFlag)
  {
    switch((pRequestCmd->CmdType) & 0xF0)
    {
    case 0xB0: pRequestCmd->AckCmdCode = 0xAB;
               if(0xB2 == pRequestCmd->CmdType)
               {
                  pRequestCmd->AckCodeH   = 0x02;
                  if(0 == gMotorMachine.RunningState)
                  {
                      gMotorMachine.OpenFlag  = 1;
                      gMotorMachine.CloseFlag = 0;
                  }
                  else
                  {
                    pRequestCmd->AckCodeL = BSP_ERROR;
                  }
               }
               break;
      
      default: state = BSP_NOCMD; break;
    
    }
    
    BSP_AckRequestCmdFromDriverBoard(pRequestCmd);
    
    pRequestCmd->HandingFlag      = 0;
    pRequestCmd->RevRequestFlag   = 0;
    pRequestCmd->DataLength       = 0;
    pRequestCmd->DataLengthHight  = 0;
    pRequestCmd->DataLengthLow    = 0;
    pRequestCmd->RevDataCnt       = 0;
    pRequestCmd->TotalLength      = 0;
  } 
  return state;
}





  /**
  * @}
  */
  /**
  * @}
  */
  /*****************************END OF FILE**************************************/
