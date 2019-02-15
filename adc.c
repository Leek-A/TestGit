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
/*
	NODE:LEEKA2
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "dac.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "macro.h"
#include "transmit_receive.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
//#include "AHRS_GetData.h"
//#include "AHRS_update.h"
#include "imu.h"
#include "imu_decoding.h"
#include "DataScope_DP.h"
#include "SuperCAP.h"
#include "Referee_Decode.h"
/* USER CODE BEGIN Includes */
#define CAP_I 2					//手动控制下电容的充电电流
#define C_P_OFFSET 6.6f			//底盘功率与裁判系统的补偿误差	
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


uint16_t cap_value=0;

float current_i[2]={0};//传感器电流值
float current_i_ex[2]={0};
uint16_t current_base[2]={0};
float senser_power[2]={0};//DC/DC的输出功率：	均值滤波 | 低通滤波
float chasis_power[2]={0};//总功率：			 	均值滤波 | 低通滤波
extern uint16_t dma_count;					//滤波计数
extern uint64_t value_sum[ADC_NUM];			//滤波求和的存储值
extern unsigned int value[ADC_NUM];			//DMA传输回来的ADC的转换值
extern unsigned int value_res[ADC_NUM];		//对应value[]的平均值:电容电压PB0 | 恒压恒流输出电流PC4 | 底盘输入电流PC5
extern unsigned int value_filter[ADC_NUM];	//对应res的低通滤波处理结果
extern extPowerHeatData_t  PowerHeatData;         //功率和热量状态
//uint32_t cap_temp;
//float cap_volt;
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
		//超级电容控制初始化
		
		/* USER CODE END SysInit */

		/* Initialize all configured peripherals */
		MX_GPIO_Init();                          //初始化控制小灯的IO和SPI使能端
		MX_DAC_Init();
    	MX_DMA_Init();                           //初始化DMA
		//MX_TIM2_Init(BREAK_100MS);               //初始化TIM2   10HZ       用于发送数据给裁判系统
		MX_TIM3_Init(BREAK_1MS);                 //初始化TIM3   1ms        用于计算系统时间
	    MX_TIM5_Init(BREAK_1MS);                 //初始化TIM5   1ms        用于控制云台           (目前程序中没有用)
    	MX_TIM6_Init(BREAK_4MS);                 //初始化TIM6   4ms        用于控制底盘
	    MX_TIM7_Init(BREAK_1MS);                 //初始化TIM7   1ms        用于解算IMU            (目前程序中没有用)
	    MX_USART1_UART_Init(DEBUS_BDR);          //初始化串口1  10w波特率  用于和遥控器通信       接收     使用DMA
 	    MX_USART6_UART_Init(NORMAL_BDR);         //初始化串口6  115200     用于和裁判系统通信     收发     使用DMA
//    	MX_USART2_UART_Init(NORMAL_BDR);         //初始化串口2  115200     蓝牙通信               收发     发送使用DMA 
	    MX_UART7_Init(NORMAL_BDR);               //初始化串口7  115200     用于和TX2通信          收发     使用DMA
	 	MX_USART3_UART_Init();
		//MX_UART8_Init(NORMAL_BDR);               //初始化串口8  115200     用于测试               收发     不使用DMA
	  	MX_CAN1_Init();                          //初始化CAN1   1MHz       控制电机
   	    CanFilter_Init(&hcan1);                  //初始化CAN1滤波器
	    MX_SPI5_Init();                          //初始化SPI    656kbits/s 和IMU通信
	    Ctrl_Init();                             //初始化控制flag
		AHRS_Init();                             //初始化AHRS对应寄存器
		AHRS_Offset_Init();                      //校准AHRS
  	    //AHRS_SolveInit();                      //初始化解算相关数据
		MX_ADC1_Init();
		HAL_Delay(3000);                         //延时等待CAN初始化
  

		/* USER CODE BEGIN 2 */
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);//开启恒压恒流
	    HAL_Delay(80);
	    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	  
	  
		HAL_TIM_Base_Start_IT(&htim3);           //开始系统计数
	    //HAL_TIM_Base_Start_IT(&htim7);           //开始IMU数据解算
	    //HAL_TIM_Base_Start_IT(&htim5);           //开始云台控制
	    //HAL_TIM_Base_Start_IT(&htim2);           //开始向裁判系统发送数据
	 	RemotreCtl_Data_Receive_Start();         //开始接收遥控器数据
	    Referee_Data_Receive_Start();            //开始接收裁判系统数据
	    HAL_TIM_Base_Start_IT(&htim6);           //开始底盘控制
	    HAL_DAC_Start(&hdac,DAC_CHANNEL_1);		//开启DAC转换
	    HAL_ADC_Start_DMA(&hadc1,value,3);		//开启ADC转换
 	  
	  
		/* USER CODE END 2 */
		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
					
		cap_current_value=HAL_DAC_GetValue(&hdac,DAC_CHANNEL_1);//读取DAC的输出
		
		//ADC读取电压值 
		GetCapVoltage();//cap_volt
	
		if(current_base[0]==0)	current_base[0]=value_res[1];
		if(current_base[1]==0)	current_base[1]=value_res[2];
		if(value_filter[1]>=current_base[0]){
			current_i[0]=(value_filter[1]-current_base[0])*33/4096.0f;//输出电流
			current_i_ex[0]=(value_res[1]-current_base[0])*33/4096.0f;//输出电流			
		}
		//else current_i[0]=0;
		if(value_filter[2]>=current_base[1]){
			current_i[1]=(value_filter[2]-current_base[1])*33/4096.0f;//输入电流
			current_i_ex[1]=(value_res[2]-current_base[1])*33/4096.0f;//输出电流
		}
		//else current_i[1]=0;
		senser_power[1]=cap_volt*current_i[0];
		chasis_power[1]=PowerHeatData.chassisVolt*current_i[1]+C_P_OFFSET+senser_power[1];
		senser_power[0]=cap_volt*current_i_ex[0];
		chasis_power[0]=PowerHeatData.chassisVolt*current_i_ex[1]+C_P_OFFSET+senser_power[0];
		
		switch(RemoteCtrlData.remote.s2){
			case 1:
			case 2:CAP_AUTO_CTRL=CAP_AUTO;break;
			case 3:CAP_AUTO_CTRL=CAP_MANUAL;break;
		}
		switch(RemoteCtrlData.remote.s1){
			case 1:CAP_CTRL=CHARGE;break;
			case 2:CAP_CTRL=TO_CAP;break;
			case 3:CAP_CTRL=TO_BATTERY;break;
		}
		if(CAP_AUTO_CTRL==CAP_MANUAL){
								
			switch(CAP_CTRL){
				case BATTERY:
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);//IN2常闭  电容 <- 恒压恒流
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);//IN1常闭  电池 -> 电机
					CAP_CTRL+=100;
					EX_CTRL=BATTERY;
					break;
				case CAP:
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//IN2开  电容 -> 电机
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);//IN1开  电池 -> 断开
					CAP_CTRL+=100;
					EX_CTRL=CAP;
					break;
				case CHARGE:
					if(EX_CTRL==BATTERY){//从电池供电-开启-》电容充电
						cap_value=1394*CAP_I+34;
						HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,cap_value);
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);//开启充电
						CAP_CTRL+=100;
						EX_CTRL=CHARGE;
					}
					break;
				case DISCHARGE:
						cap_value=0;
						HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,cap_value);
						CAP_CTRL+=100;
						EX_CTRL=DISCHARGE;
					break;
				case TO_CAP:
					if(cap_current_value!=0){
						HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);//停止电容充电
					}
					if(EX_CTRL==BATTERY){//从电池供电-切换-》电容供电
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
						HAL_Delay(2);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
						CAP_CTRL+=100;
						EX_CTRL=CAP;
					}
					
					break;
				case TO_BATTERY:
					if(cap_current_value!=0){
						HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);//停止电容充电
					}
					if(EX_CTRL==CHARGE){//从电容充电-切换-》电池供电
						cap_value=0;
						HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,cap_value);
						CAP_CTRL+=100;
						//EX_CTRL=BATTERY;
					}else if(EX_CTRL==CAP){//从电容供电-切换-》电池供电
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
						HAL_Delay(2);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
						CAP_CTRL+=100;
						//EX_CTRL=BATTERY;
					}
					EX_CTRL=BATTERY;
					break;
				default: break;
			}
		/* USER CODE BEGIN 3 */
		}//IF (CAP_AUTO_CTRL==CAP_MANUAL)
		else{
			HAL_Delay(2);
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
