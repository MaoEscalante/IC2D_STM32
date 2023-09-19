
//https://www.st.com/resource/en/datasheet/stm32f427vg.pdf

#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>



_FC2D_handler FC2D_handler;
_FC2D FC2D = {0};
bool PA_PRESSED = false;

/*
*	LEFT:
*		Enc_B  : PA9
*		Vout   : PA5
*		Force  : PC1
*
*	RIGHT:
*		Enc_A : PC11
*		Enc_B : PC12
*
*/


#define LCD_FRAME_BUFFER_LAYER0                  (LCD_FRAME_BUFFER+0x130000)
#define LCD_FRAME_BUFFER_LAYER1                  LCD_FRAME_BUFFER

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

void init_FC2D();
void init_ADC_FC2D();


int main(void)
{

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	osKernelInitialize();

	init_FC2D();
	init_ADC_FC2D();
	configure_DAC();

	HAL_DAC_Start(&DAC_Config, DAC_CHANNEL_2);

	MX_FREERTOS_Init();
	osKernelStart();


	while (1){

	}
}


void init_ADC_FC2D(){

	GPIO_InitTypeDef ADCpin; //create an instance of GPIO_InitTypeDef C struct
	ADCpin.Pin = GPIO_PIN_1;
	ADCpin.Mode = GPIO_MODE_ANALOG; // Select Analog Mode
	ADCpin.Pull = GPIO_NOPULL; // Disable internal pull-up or pull-down resistor
	HAL_GPIO_Init(GPIOC, &ADCpin);

	__HAL_RCC_ADC2_CLK_ENABLE();

	FC2D_handler.ADC_u.Instance = ADC2;

	FC2D_handler.ADC_u.Init.Resolution = ADC_RESOLUTION_12B;
	FC2D_handler.ADC_u.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	FC2D_handler.ADC_u.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	HAL_ADC_Init(&FC2D_handler.ADC_u);


	ADC_ChannelConfTypeDef Channel_AN0; // create an instance of ADC_ChannelConfTypeDef
	Channel_AN0.Channel = ADC_CHANNEL_11; // select analog channel 0
	Channel_AN0.Rank = 1; // set rank to 1
	Channel_AN0.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	HAL_ADC_ConfigChannel(&FC2D_handler.ADC_u, &Channel_AN0);
}

void init_FC2D(){
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

	FC2D.actuator[0].encoder.GPIO_PIN_A = GPIO_PIN_14;
	FC2D.actuator[0].encoder.GPIO_PIN_B = GPIO_PIN_15;
	FC2D.actuator[0].encoder.PORT_PIN_A = GPIOC;
	FC2D.actuator[0].encoder.PORT_PIN_B = GPIOC;
	FC2D.actuator[0].encoder.IRQn_A     = EXTI15_10_IRQn;
	FC2D.actuator[0].encoder.IRQn_B     = EXTI15_10_IRQn;
	FC2D.actuator[0].encoder.resolution = 1.0/4096.0f;

	FC2D.actuator[1].encoder.GPIO_PIN_A = GPIO_PIN_11;
	FC2D.actuator[1].encoder.GPIO_PIN_B = GPIO_PIN_12;
	FC2D.actuator[1].encoder.PORT_PIN_A = GPIOC;
	FC2D.actuator[1].encoder.PORT_PIN_B = GPIOC;
	FC2D.actuator[1].encoder.IRQn_A     = EXTI15_10_IRQn;
	FC2D.actuator[1].encoder.IRQn_B     = EXTI15_10_IRQn;
	FC2D.actuator[1].encoder.resolution = 1.0/4096.0f;

	BSP_LCD_Init();

	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER1);

	BSP_LCD_SelectLayer(1);

	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetColorKeying(1, LCD_COLOR_WHITE);
	BSP_LCD_SetLayerVisible(1, ENABLE);




	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	LG_INIT_ENCODER(&FC2D.actuator[0].encoder);
	LG_INIT_ENCODER(&FC2D.actuator[1].encoder);
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
		RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

		__HAL_RCC_PWR_CLK_ENABLE();
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
		RCC_OscInitStruct.HSEState = RCC_HSE_ON;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
		RCC_OscInitStruct.PLL.PLLM = 8;
		RCC_OscInitStruct.PLL.PLLN = 360;
		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
		RCC_OscInitStruct.PLL.PLLQ = 7;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
			Error_Handler();
		}

		if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
			Error_Handler();
		}

		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
			Error_Handler();
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

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
