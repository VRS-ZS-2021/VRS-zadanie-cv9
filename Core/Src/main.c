/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "display.h"
#include"iks01a1.h"
#include"stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t mode, error;
extern uint64_t disp_time;
uint64_t saved_time;
char display_text [100];
int act_index=0, right=1,index_for_dot=0;
float humidity, temperature, pressure, altitude;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */
  /*SYSCFG->EXTICR[1] &= ~(0xFU);
  SYSCFG->EXTICR[1] |= (0x1U);
  EXTI->IMR |= EXTI_IMR_MR4;
  EXTI->RTSR &= ~(EXTI_IMR_MR4);
  EXTI->FTSR |= EXTI_IMR_MR4;*/
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Systick_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  error = 0;
  if(!iks01a1_init()) error = 1;


  setSegments();
  setDigits();
  LL_mDelay(2000);
  resetDigits();
  resetSegments();

  mode = 0;
//  strcpy(display_text,"0123456789\0");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(error) {
		  strcpy(display_text,"I2C_who_am_I_error\0");
	  } else {
		  hts221_start_measurement();
		  lps25hb_start_measurement();
		  hts221_get_humidity(&humidity);
		  hts221_get_temperature(&temperature);
		  lps25hb_get_pressure(&pressure);
		  lps25hb_start_measurement();
		  lps25hb_get_altitude(&altitude);

		  if (mode == 0) {
			  if(temperature>=100){
				temperature=99.9;
			  }
			  if(temperature<=-100){
				temperature=-99.9;
			  }
			  sprintf(display_text, "TEMP_%2.1f", temperature);

		      for(int i=0; i<strlen(display_text); i++){
		          if(display_text[i]=='.'){
		              display_text[i] = display_text[i+1];
		              display_text[i+1] = '\0';
		          }
		      }
		      index_for_dot=strlen(display_text)-1;
		   }

		   if (mode == 1){
			  if(humidity>=100){
				humidity=99;
			  }
			  if(humidity<0){
				humidity = 0;
			  }
			  sprintf(display_text, "HUM_%2.0f", humidity);
		   }


		   if (mode == 2){
			  if(pressure>=10000){
				pressure=9999.99;
			  }
			  if(pressure<0){
				pressure = 0;
			  }
			  sprintf(display_text, "BAR_%4.2f", pressure);

		      for(int i=0; i<strlen(display_text); i++){
		          if(display_text[i]=='.'){
		              display_text[i] = display_text[i+1];
		              display_text[i+1] = display_text[i+2];
		              display_text[i+2] = '\0';
		          }
		      }
		      index_for_dot=strlen(display_text)-2;
		   }

		   if(mode == 3){
			  if(altitude>=10000){
				altitude = 9999.9;
			  }
			  if(altitude<=-10000){
				altitude = -9999.9;
			  }
			  sprintf(display_text, "ALT_%4.1f", altitude);

		      for(int i=0; i<strlen(display_text); i++){
		          if(display_text[i]=='.'){
		              display_text[i] = display_text[i+1];
		              display_text[i+1] = '\0';
		          }
		      }
		      index_for_dot=strlen(display_text)-1;
		   }
	  }

	  if(disp_time > (saved_time + 500))
	  	  {

	  		  display_sign(display_text[act_index],display_text[act_index+1],display_text[act_index+2],display_text[act_index+3],index_for_dot,act_index+3);
	  	  	  saved_time = disp_time;

	  	  	  if(right){ //urcovanie smeru
	  	  		  act_index++;
	  	  	  } else {
	  	  		  act_index--;
	  	  	  }
	  	  	  if(act_index == strlen(display_text)-4) {
	  	  		  right = 0;
	  	  	  }
	  	  	  if (act_index == 0){
	  	  		  right = 1;
	  	  	  }

	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */
uint8_t checkButtonState(GPIO_TypeDef* PORT, uint8_t PIN, uint8_t edge, uint16_t samples_window, uint16_t samples_required)
{
	//type your code for "checkButtonState" implementation here:
	uint16_t detection = 0, timeout = 0;
	while(timeout <= samples_window){ // cyklus bezi pokial nedocita potrebny pocet vzoriek a ak nahodou nastane detekcia funkcia sa ukonci a vrati 1
		uint8_t actual_value = BUTTON_READ_VALUE;
		if((actual_value && edge) || (!(actual_value) && !(edge))) {
			detection++;
		}
		else{
			detection = 0;
		}

		timeout++;

		if(detection == samples_required){
			return 1;
		}
	}
	if (((timeout > samples_window) && (detection != samples_required))){ //ak cyklus dobehol a nenapocitali sme dostatocny pocet vzoriek iducich po sebe vrati 0
		return 0;
	}
	return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
