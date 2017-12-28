/**
******************************************************************************
  * File Name          : bsp_motor.c
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
#include "bsp_motor.h"
extern MOTORMACHINE gMotorMachine;
extern GPIOSTATUSDETECTION gGentleSensorStatusDetection;

/*******************************************************************************
*
*       Function        :BSP_MotorInit()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorInit(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  
  HAL_Delay(100);
  
  gMotorMachine.RunningState     = 0;
  gMotorMachine.RunDir           = UPDIR;
  gMotorMachine.HorFilterCnt     = 0;
  gMotorMachine.VerFilterCnt     = 0;
  gMotorMachine.EncounteredFlag  = 0;
  gMotorMachine.Motor_Error      = Motor_OK;
  gMotorMachine.RemoteControFlag = 0;
  
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorOpen()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorOpen(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);  
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorClose()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorClose(void)
{
  BSP_StatusTypeDef state  = BSP_OK;  
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorRun()
*
*       Input           :uint8_t nDir
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorRun(uint8_t nDir)
{
  BSP_StatusTypeDef state  = BSP_OK;
  if(UPDIR == nDir) //down
  {
    HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);
  }
  
  if(DOWNDIR == nDir) //up
  {
    HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);    
  }
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorStop()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorStop(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorCheck()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorCheck(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  
  if(gMotorMachine.RemoteControFlag)
  {
    return state;
  }
  
  if(gMotorMachine.VerticalRasterState && 0 == gMotorMachine.HorizontalRasterState)
  {
    if(1 == gMotorMachine.CloseFlag)
    {
      if(DOWNDIR == gMotorMachine.RunDir)
      {
        gMotorMachine.StartFlag = 1;
        return state;
      }
    }
    
    if(0 == gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
    {
      if(1 == gMotorMachine.OpenFlag)
      {
        if( 1 == gMotorMachine.GentleSensorFlag )
        {
          return state;
        }
        gMotorMachine.OpenFlag  = 0;
        gMotorMachine.CloseFlag = 1;
        return state;
      }
    }
  }
  
  if(0 == gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
  {
    if(gMotorMachine.OpenFlag)
    {
      if(UPDIR == gMotorMachine.RunDir)
      {
        gMotorMachine.StartFlag = 1;
        return state;
      }
    }
    if(gMotorMachine.CloseFlag)
    {
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.OpenFlag  = 0;
      return state;
    }
  }
  
  if(gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
  {
    if(gMotorMachine.OpenFlag)
    {
      if(0 == gMotorMachine.RunningState)
      {
        gMotorMachine.RunDir = UPDIR;
        gMotorMachine.OpenFlag = 0;
        gMotorMachine.CloseFlag = 1;
        gMotorMachine.StartFlag = 1;
        return state;
      }
    }
    if(1 == gMotorMachine.CloseFlag)
    {
      if(gMotorMachine.RunningState)
      {
        if(gMotorMachine.GentleSensorFlag && gGentleSensorStatusDetection.GpioCheckedFlag)
        {
          BSP_MotorStop();
          gMotorMachine.RunningState = 0;
          gMotorMachine.OpenFlag = 1;
          gGentleSensorStatusDetection.GpioCheckedFlag = 0;
          return state;
        }
      }
      else
      {
        gMotorMachine.RunDir = DOWNDIR;
        gMotorMachine.StartFlag = 1;
        return state;
      }
    }
  
  }
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorAction()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorAction(void)
{
  BSP_StatusTypeDef state = BSP_OK;
  if(gMotorMachine.StartFlag)
  {
    if(gMotorMachine.RunningState)
    {
      gMotorMachine.StartFlag = 0;
      return state;
    }
    gMotorMachine.RunningState = 1;
    BSP_MotorRun(gMotorMachine.RunDir);
    gMotorMachine.StartFlag = 0;
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
