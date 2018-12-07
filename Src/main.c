
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32f0xx_hal.h"
#include "tm_stm32_i2c.h"
#include "tm_stm32_mpu6050.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include <math.h>
#include<string.h>
#include <stdbool.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
	uint8_t device_address = 0xD0;
	uint8_t register_address=0x3B;
	uint8_t Data[14];
	char bufer[150];
	uint16_t paket_id;
	float AX,AY,AZ,T,GX,GY,GZ;
	float XAxisFinal,YAxisFinal,ZAxisFinal,XGyroFinal,YGyroFinal,ZGyroFinal;
	float delta_angle_x,delta_angle_y,pitch,roll,yaw,real_angle = 0,prev_angle = 0;
	float ang_x,ang_y,ang_z,Gangle_x,Gangle_y;
	int AX_offset = 0,AY_offset = 0,AZ_offset = 1700;
	float old_x = 0,old_y = 0,prev_angle_x = 0,prev_angle_y = 0;
	int GX_offset = 520,GY_offset = 0,GZ_offset = 0;
	float XAxis_kalman,YAxis_kalman,ZAxis_kalman,XGyro_kalman,YGyro_kalman,ZGyro_kalman;
	float kalman_old = 0,cov_old = 1,kalman_new,cov_new,avg,val1, val2, val3, val4, val5 ;
	int med,m,med_sort[5],c_avg = 1, c_med = 1, d = 0, med1_sort, med2_sort, med3_sort, med4_sort;
	TM_MPU6050_t MPU6050_Sensor;

	typedef union {
	    float f;         // Assigning fVal.f will also populate fVal.bytes;
	    char bytes[4];   // Both fVal.f and fVal.bytes share the same 4 bytes of memory.
	} fVal;
	char packed[44];   // Some arbitrary message, note the size.
	fVal packedFloatGX,packedFloatGY,packedFloatGZ,packedFloatAX,packedFloatAY,packedFloatAZ,packedFloatT,packedFloatRoll,packedFloatPitch,packedFloatYaw;
	float unpacked;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	I2C_HandleTypeDef hi2c1;
		TIM_HandleTypeDef htim2;
/* USER CODE END PV */
		/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void read_MPU6050_data();
void read_Accolemetre();
void read_Gyroscope();
float mov_avg (int counter, float input);
float kalman_filter (float input);
float median (int counter, int input);
void sort(int a[], int size);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start_IT(&htim2);
    TM_MPU6050_Init(&MPU6050_Sensor, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_2G, TM_MPU6050_Gyroscope_250s);
    HAL_I2C_IsDeviceReady(&hi2c1,device_address,1,1000);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
// Read bytes off serial connection
		  // Overite bytes of union with float variable
/* USER CODE BEGIN 4 */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{

		if(htim->Instance == TIM2)
		{
			c_avg = c_avg + 1;
			    c_med = c_med + 1;

			    if (c_avg > 5 && c_med > 5)
			    {
			      c_avg = 6;
			      c_med = 6;
			    }
			read_MPU6050_data();
			read_Accolemetre();
			read_Gyroscope();
			pitch = 180 * atan (XAxisFinal/sqrt(YAxisFinal*YAxisFinal + ZAxisFinal*ZAxisFinal))/M_PI;
			roll = 180 * atan (YAxisFinal/sqrt(XAxisFinal*XAxisFinal + ZAxisFinal*ZAxisFinal))/M_PI;
			yaw = 180 * atan (ZAxisFinal/sqrt(XAxisFinal*XAxisFinal + ZAxisFinal*ZAxisFinal))/M_PI;


			paket_id++;
			packed[0]=0x53;
			packed[1]=paket_id;
			packed[2]=0x12;
			packedFloatGX.f = XGyro_kalman;
			packed[3] = packedFloatGX.bytes[0];
			packed[4] = packedFloatGX.bytes[1];
			packed[5] = packedFloatGX.bytes[2];
			packed[6] = packedFloatGX.bytes[3];

			packedFloatGY.f = YGyro_kalman;
			packed[7] = packedFloatGY.bytes[0];
			packed[8] = packedFloatGY.bytes[1];
			packed[9] = packedFloatGY.bytes[2];
			packed[10] = packedFloatGY.bytes[3];

			packedFloatGZ.f = ZGyro_kalman;
			packed[11] = packedFloatGZ.bytes[0];
			packed[12] = packedFloatGZ.bytes[1];
			packed[13] = packedFloatGZ.bytes[2];
			packed[14] = packedFloatGZ.bytes[3];

			packedFloatAX.f = XAxis_kalman;
			packed[15] = packedFloatAX.bytes[0];
			packed[16] = packedFloatAX.bytes[1];
			packed[17] = packedFloatAX.bytes[2];
			packed[18] = packedFloatAX.bytes[3];

			packedFloatAY.f = YAxis_kalman;
			packed[19] = packedFloatAY.bytes[0];
			packed[20] = packedFloatAY.bytes[1];
			packed[21] = packedFloatAY.bytes[2];
			packed[22] = packedFloatAY.bytes[3];

			packedFloatAZ.f = ZAxis_kalman;
			packed[23] = packedFloatAZ.bytes[0];
			packed[24] = packedFloatAZ.bytes[1];
			packed[25] = packedFloatAZ.bytes[2];
			packed[26] = packedFloatAZ.bytes[3];

			packedFloatT.f = T;
			packed[27] = packedFloatT.bytes[0];
			packed[28] = packedFloatT.bytes[1];
			packed[29] = packedFloatT.bytes[2];
			packed[30] = packedFloatT.bytes[3];

			packedFloatPitch.f = pitch;
			packed[31] = packedFloatPitch.bytes[0];
			packed[32] = packedFloatPitch.bytes[1];
			packed[33] = packedFloatPitch.bytes[2];
			packed[34] = packedFloatPitch.bytes[3];

			packedFloatYaw.f = yaw;
			packed[35] = packedFloatYaw.bytes[0];
			packed[36] = packedFloatYaw.bytes[1];
			packed[37] = packedFloatYaw.bytes[2];
			packed[38] = packedFloatYaw.bytes[3];

			packedFloatRoll.f = roll;
			packed[39] = packedFloatRoll.bytes[0];
			packed[40] = packedFloatRoll.bytes[1];
			packed[41] = packedFloatRoll.bytes[2];
			packed[42] = packedFloatRoll.bytes[3];
			packed[43]=0x45;

			HAL_UART_Transmit_IT(&huart1, (uint8_t*)packed,sizeof(packed));

			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9);
		}
	}
/* USER CODE END 4 */
	float mov_avg (int counter, float input)
	{
	  avg = 0;
	  m = 0;

	  if (counter == 1) {
	    val1 = input;
	    avg = val1;

	  }
	  else if (counter == 2) {
	    val2 = input;
	    avg = val2;

	  }
	  else if (counter == 3) {
	    val3 = input;
	    avg = val3;

	  }
	  else if (counter == 4) {
	    val4 = input;
	    avg = val4;

	  }
	  else if (counter == 5) {
	    val5 = input;
	    avg = val5;

	  }
	  else if (counter > 5 ) {

	    counter = 6;

	    if (val1 == 0) {
	      m = m + 1;
	    }
	    if (val2 == 0) {
	      m = m + 1;
	    }
	    if (val3 == 0) {
	      m = m + 1;
	    }
	    if (val4 == 0) {
	      m = m + 1;
	    }
	    if (val5 == 0) {
	      m = m + 1;
	    }
	    if (input == 0) {
	      m = m + 1;
	    }

	    d = 6 - m;


	    if (d == 0)
	    {
	      avg = input;
	      counter = 1;
	    }
	    else
	    {
	      avg = (val1 + val2 + val3 + val4 + val5 + input) / d;
	    }

	    val1 = val2;
	    val2 = val3;
	    val3 = val4;
	    val4 = val5;
	    val5 = input;
	  }

	  return avg;
	}

	float median (int counter, int input)
	{
	  if (counter == 1) {
	    med1_sort = input;
	    med_sort[0] = med1_sort;

	  }
	  else if (counter == 2) {
	    med2_sort = input;
	    med_sort[1] = med2_sort;

	  }
	  else if (counter == 3) {
	    med3_sort = input;
	    med_sort[2] = med3_sort;

	  }
	  else if (counter == 4) {
	    med4_sort = input;
	    med_sort[3] = med4_sort;

	  }

	  else if (counter >= 5) {

	    counter = 6;

	    med_sort[4] = input;

	    sort(med_sort, 5);

	    med = med_sort[2];

	    med1_sort = med2_sort;
	    med2_sort = med3_sort;
	    med3_sort = med4_sort;
	    med4_sort = input;

	    med_sort[0] = med1_sort;
	    med_sort[1] = med2_sort;
	    med_sort[2] = med3_sort;
	    med_sort[3] = med4_sort;

	  }
	  return med;
	}

	void sort(int a[], int size) {
	  for (int i = 0; i < (size - 1); i++) {
	    for (int o = 0; o < (size - (i + 1)); o++) {
	      if (a[o] > a[o + 1]) {
	        int t = a[o];
	        a[o] = a[o + 1];
	        a[o + 1] = t;
	      }
	    }
	  }
	}
	float kalman_filter (float input)
	{

	   kalman_new = kalman_old;
	  cov_new = cov_old + 0.50;

	  float kalman_gain = cov_new / (cov_new + 0.9);
	  float kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new));

	  cov_new = (1 - kalman_gain) * cov_old;
	  cov_old = cov_new;

	  kalman_old = kalman_calculated;

	  return kalman_calculated;

	}
	void read_MPU6050_data()
	{

		HAL_I2C_Master_Transmit(&hi2c1,device_address,&register_address,1,1000);

		HAL_I2C_Master_Receive(&hi2c1,device_address,Data,14,1000);

		AX=(float)(((int16_t)(Data[0]<<8 | Data[1]))+AX_offset);
		AY=(float)(((int16_t)(Data[2]<<8 | Data[3]))+AY_offset);
		AZ=(float)(((int16_t)(Data[4]<<8 | Data[5]))+AZ_offset);
		T=(float)(((int16_t)(Data[6]<<8 | Data[7]))/(float)340+(float)35);
		GX=(float)(((int16_t)(Data[8]<<8 | Data[9]))+GX_offset);
		GY=(float)(((int16_t)(Data[10]<<8 | Data[11]))+GY_offset);
		GZ=(float)(((int16_t)(Data[12]<<8 | Data[13]))+GZ_offset);
	}

	void read_Accolemetre()
	{
		XAxisFinal = (float) AX / 16384.0;
		YAxisFinal = (float) AY / 16384.0;
		ZAxisFinal = (float) AZ / 16384.0;

		XAxis_kalman = kalman_filter(XAxisFinal);
		YAxis_kalman = kalman_filter(YAxisFinal);
		ZAxis_kalman = kalman_filter(ZAxisFinal);

		if(XAxis_kalman>0.99) XAxis_kalman=1;   //0.99 olan deðerler 1'e tamamlanýr.
		if(YAxis_kalman>0.99) YAxis_kalman=1;
		if(ZAxis_kalman>0.99) ZAxis_kalman=1;

		if(XAxis_kalman<-0.99) XAxis_kalman=-1; //-0.99 olan deðerler 1'e tamamlanýr.
		if(YAxis_kalman<-0.99) YAxis_kalman=-1;
		if(ZAxis_kalman<-0.99) ZAxis_kalman=-1;

		ang_x = atan(AY/(sqrt(pow(AX,2)+pow(AZ,2)))) * 57296 / 1000; //Euler Açý formülüne göre açý hesabý. (X-Ekseni)
		ang_y = atan(AY/(sqrt(pow(AX,2)+pow(AZ,2)))) * 57296 / 1000;
	}

	void read_Gyroscope()
	{
		XGyroFinal = (float)GX / 131;                         //Datasheet'te yazan deðerlere göre "deg/s" cinsinden açýsal hýz bulunur. (X ekseni için)
		YGyroFinal = (float)GY / 131;                         //Datasheet'te yazan deðerlere göre "deg/s" cinsinden açýsal hýz bulunur. (Y ekseni için)
		ZGyroFinal = (float)GZ / 131;

		XGyro_kalman = kalman_filter(XGyroFinal);
		YGyro_kalman = kalman_filter(YGyroFinal);
		ZGyro_kalman = kalman_filter(ZGyroFinal);

		delta_angle_x = (0.03 * old_x) + ((0.03 * (XGyro_kalman - old_x)) / 2); //açýsal hýz ve geçen süreye baðlý olarak taranan açý hesaplanýr
		Gangle_x = (prev_angle_x + delta_angle_x);                             //taranan açý deðeri ile bir önceki açý deðeri hesaplanarak
		prev_angle_x = Gangle_x;                                                     //güncel açý deðeri bir sonraki döngüde kullanýlmak üzere önceki açý deðeri olarak kaydedilir
		old_x = XGyro_kalman;

		delta_angle_y = (0.03 * old_y) + ((0.03 * (YGyro_kalman - old_y)) / 2); //yukarýdaki iþlemlerin aynýsý Y ekseni için de yapýlýr
		Gangle_y = (prev_angle_y + delta_angle_y);
		prev_angle_y = Gangle_y;
		old_y = YGyro_kalman;

	}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
