#include <stm32f4xx_lg_defs.h>

DAC_HandleTypeDef DAC_Config = {0};

void configure_GPIO(void)
{

/* DAC GPIO ---> PA5 */
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 GPIO_InitStruct.Pin = GPIO_PIN_5;
 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void configure_DAC(void)
{
 DAC_ChannelConfTypeDef Channel_Config;
 __HAL_RCC_DAC_CLK_ENABLE();
 /* DAC Initialization */
 DAC_Config.Instance = DAC;

 HAL_DAC_Init(&DAC_Config);

 /* DAC channel_OUT2 config */
 Channel_Config.DAC_Trigger = DAC_TRIGGER_NONE;
 Channel_Config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
 HAL_DAC_ConfigChannel(&DAC_Config, &Channel_Config, DAC_CHANNEL_1);
}
