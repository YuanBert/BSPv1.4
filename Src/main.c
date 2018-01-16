/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_protocol.h"
#include "bsp_gentlesensor.h"
#include "bsp_common.h"
#include "bsp_motor.h"
#include "bsp_led.h"
#include "BSP_DAC5571.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern  USARTRECIVETYPE     DriverBoardUsartType;

MOTORMACHINE gMotorMachine;
PROTOCOLCMD  gDriverBoardProtocolCmd;
GPIOSTATUSDETECTION gGentleSensorStatusDetection;
GPIOSTATUSDETECTION gRadarInputStatusGpio;
GPIOSTATUSDETECTION gMCUAIRInputStatusGpio;

uint8_t		gCtrlSpeedTimFlag;
uint8_t		gCtrlSpeedTimCnt;
uint16_t	gCtrlSpeedCnt;

uint8_t         gComingCarFlag;
uint32_t        gWaitCnt;

uint8_t gVerLastReadVal;
uint8_t gVerCurrentReadVal;
uint8_t gHorLastReadVal;
uint8_t gHorCurrentReadVal;

uint16_t    gTIM4Cnt;
uint8_t     gTIM4CntFlag;

uint16_t    gTIM5Cnt;
uint8_t     gTIM5CntFlag;
uint16_t    gTIM5LedCnt;
uint8_t     gTIM5LedFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  gRadarInputStatusGpio.GpioFilterCntSum  = 10;
  gMCUAIRInputStatusGpio.GpioFilterCntSum = 5;
  
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  BSP_MotorInit();
  BSP_DriverBoardProtocolInit();
  BSP_RUNNINGLED_ON();
  BSP_DAC5571_Init(NormalOperationMode);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    BSP_MotorCheck();
    BSP_MotorAction();
    
    if(gMotorMachine.RunningState && UPDIR == gMotorMachine.RunDir)
    {
      if(gCtrlSpeedTimFlag)
      {
        gCtrlSpeedTimFlag = 0;
        BSP_DAC5571_Check();
      }
    }
    
    if(gMotorMachine.RunningState && DOWNDIR == gMotorMachine.RunDir)
    {
       BSP_DAC5571_WriteValue(NormalOperationMode, 0x9F);
    }
    
//    if(gTIM5CntFlag)
//    {
//      if(0 == gMotorMachine.RunningState)
//      {
//        BSP_MotorRun(gMotorMachine.RunDir);
//        gMotorMachine.RunningState = 1;
//      }
//      gTIM5CntFlag = 0;
//    }
    
    BSP_HandingUartDataFromDriverBoard();
    BSP_HandingCmdFromDriverBoard(&gDriverBoardProtocolCmd);
    
    if(gTIM5LedFlag)
    {
      BSP_LEDCheck();
      gTIM5LedFlag = 0;
    }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* I2C2_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  /* 0.1 ms*/
  if(htim4.Instance == htim->Instance)
  {
    gVerCurrentReadVal = HAL_GPIO_ReadPin(VerRasterInput_GPIO_Port, VerRasterInput_Pin);
    gHorCurrentReadVal = HAL_GPIO_ReadPin(HorRasterInput_GPIO_Port, HorRasterInput_Pin);
    
    if(0 == gVerCurrentReadVal && 0 == gVerLastReadVal)
    {
      gMotorMachine.VerFilterCnt ++;
      if(gMotorMachine.VerFilterCnt > 20)
      {
        gMotorMachine.VerticalRasterState = 1;
        gMotorMachine.VerFilterCnt = 0;
        if(UPDIR == gMotorMachine.RunDir)
        {
          gMotorMachine.RunDir = DOWNDIR;
          gMotorMachine.RunningState = 0;
          BSP_MotorStop();
        }
      }
    }
    else
    {
      gMotorMachine.VerticalRasterState = 0;
      gMotorMachine.VerFilterCnt = 0;
    }
    
    if(0 == gHorCurrentReadVal && 0 == gHorLastReadVal)
    {
      gMotorMachine.HorFilterCnt ++;
      if(gMotorMachine.HorFilterCnt > 20)
      {
        gMotorMachine.HorizontalRasterState = 1;
        gMotorMachine.HorFilterCnt = 0;
        if(DOWNDIR == gMotorMachine.RunDir)
        {
          gMotorMachine.RunDir = UPDIR;
          gMotorMachine.RunningState = 0;
          gCtrlSpeedCnt = 0;//水平到位
          BSP_MotorStop();
        }
      }
    }
    else
    {
      gMotorMachine.HorizontalRasterState = 0;
      gMotorMachine.HorFilterCnt = 0;
    }
    gVerLastReadVal     = gVerCurrentReadVal;
    gHorLastReadVal     = gHorCurrentReadVal;
  }
  
  /* 1ms */
  if(htim5.Instance == htim->Instance)
  {
    
    gCtrlSpeedTimCnt++;
    if(gCtrlSpeedTimCnt > 4)
    {
      gComingCarFlag = 1;
      gCtrlSpeedTimCnt = 0;
    }
    
    gTIM5Cnt++;
    if(0 == (gTIM5Cnt%250))
    {
      gTIM5LedFlag = 1;
    }
    
    if(gTIM5Cnt > 5000)
    {
      gTIM5CntFlag = 1;
      gTIM5Cnt = 0;
    }
    /* 压力波检测 */
    gMCUAIRInputStatusGpio.GpioCurrentReadVal = HAL_GPIO_ReadPin(MCU_AIR_GPIO_Port,MCU_AIR_Pin);
    if(0 == gMCUAIRInputStatusGpio.GpioCurrentReadVal && 0 == gMCUAIRInputStatusGpio.GpioLastReadVal )
    {
      if(0 == gMCUAIRInputStatusGpio.GpioCheckedFlag)
      {
        gMCUAIRInputStatusGpio.GpioFilterCnt ++;
        if(gMCUAIRInputStatusGpio.GpioFilterCnt > gMCUAIRInputStatusGpio.GpioFilterCntSum && 0 == gMCUAIRInputStatusGpio.GpioStatusVal)
        {
          gMotorMachine.AirSensorFlag = 1;
          gMCUAIRInputStatusGpio.GpioStatusVal = 1;
          gMCUAIRInputStatusGpio.GpioFilterCnt = 0;
          gMCUAIRInputStatusGpio.GpioCheckedFlag = 1;
        }
      }
    }
    else
    {
      gMotorMachine.AirSensorFlag = 0;
      gMCUAIRInputStatusGpio.GpioCheckedFlag = 0;
      gMCUAIRInputStatusGpio.GpioStatusVal = 0;
      gMCUAIRInputStatusGpio.GpioSendDataFlag = 1;
    }
    
    /* 雷达检测 */
    gRadarInputStatusGpio.GpioCurrentReadVal = HAL_GPIO_ReadPin(RadarInput_GPIO_Port, RadarInput_Pin);
    if(0 == gRadarInputStatusGpio.GpioCurrentReadVal && 0 == gRadarInputStatusGpio.GpioLastReadVal)
    {
      if(0 == gRadarInputStatusGpio.GpioCheckedFlag)
      {
        gRadarInputStatusGpio.GpioFilterCnt ++;
        if(gRadarInputStatusGpio.GpioFilterCnt > gRadarInputStatusGpio.GpioFilterCntSum && 0 == gRadarInputStatusGpio.GpioStatusVal)
        {
          gRadarInputStatusGpio.GpioStatusVal = 1;
          gRadarInputStatusGpio.GpioFilterCnt = 0;
          gRadarInputStatusGpio.GpioCheckedFlag = 1;
          gMotorMachine.RadarSensorFlag = 1;
        }
      }
    }
    else
    {
      gMotorMachine.RadarSensorFlag = 0;
      gRadarInputStatusGpio.GpioCheckedFlag = 0;
      gRadarInputStatusGpio.GpioStatusVal = 0;
      gRadarInputStatusGpio.GpioSendDataFlag = 1;
    }
    gRadarInputStatusGpio.GpioLastReadVal = gRadarInputStatusGpio.GpioCurrentReadVal;
    
    /* 地感检测 */
    gGentleSensorStatusDetection.GpioCurrentReadVal = HAL_GPIO_ReadPin(GentleSensor_GPIO_Port,GentleSensor_Pin);
    if(0 == gGentleSensorStatusDetection.GpioCurrentReadVal && 0 == gGentleSensorStatusDetection.GpioLastReadVal)
    {
      if(0 == gGentleSensorStatusDetection.GpioCheckedFlag)
      {
        gGentleSensorStatusDetection.GpioFilterCnt ++;
        if(gGentleSensorStatusDetection.GpioFilterCnt > gGentleSensorStatusDetection.GpioFilterCntSum && 0 == gGentleSensorStatusDetection.GpioStatusVal)
        {
          gGentleSensorStatusDetection.GpioStatusVal = 1;
          gGentleSensorStatusDetection.GpioFilterCnt = 0;
          gGentleSensorStatusDetection.GpioCheckedFlag = 1;
          gMotorMachine.GentleSensorFlag = 1;
        }
      }
    }
    else
    {
      
      //车离开之后，清空标记位
      if(gGentleSensorStatusDetection.GpioStatusVal && gComingCarFlag)
      {
        gComingCarFlag = 0;
      }
      
      gMotorMachine.GentleSensorFlag = 0;
      gGentleSensorStatusDetection.GpioCheckedFlag       = 0;
      gGentleSensorStatusDetection.GpioFilterCnt     = 0;
      gGentleSensorStatusDetection.GpioStatusVal     = 0;
      gGentleSensorStatusDetection.GpioSendDataFlag  = 1;
    }     
    gGentleSensorStatusDetection.GpioLastReadVal = gGentleSensorStatusDetection.GpioCurrentReadVal; 
    
    if(gGentleSensorStatusDetection.GpioValidLogicTimeCnt > 80)
    {
      gGentleSensorStatusDetection.GpioValidLogicTimeCnt--;
    }
    
    
    //如果100s内车未通过，自动关闸
    if(gComingCarFlag)
    {
      gWaitCnt ++;
      if(gWaitCnt > 100000)
      {
        gWaitCnt = 0;
        gComingCarFlag = 0;
      }
    }
  }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
