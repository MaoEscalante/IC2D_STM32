
#include <stm32f4xx_lg_defs.h>

void LG_ENCODER_COUNTER(LG_ENCODER * encoder);

void LG_ENABLE_GPIO_CLK(GPIO_TypeDef  *GPIOx){
		  if(GPIOx == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
	 else if(GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
	 else if(GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
	 else if(GPIOx == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
}
void LG_DISABLE_GPIO_CLK(GPIO_TypeDef  *GPIOx){
		  if(GPIOx == GPIOA) __HAL_RCC_GPIOA_CLK_DISABLE();
	 else if(GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_DISABLE();
	 else if(GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_DISABLE();
	 else if(GPIOx == GPIOH) __HAL_RCC_GPIOH_CLK_DISABLE();
}

void LG_INIT_ENCODER(LG_ENCODER * encoder){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	LG_ENABLE_GPIO_CLK(encoder->PORT_PIN_A);
	LG_ENABLE_GPIO_CLK(encoder->PORT_PIN_B);

	GPIO_InitStruct.Pin = encoder->GPIO_PIN_A;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	HAL_GPIO_Init(encoder->PORT_PIN_A, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(encoder->IRQn_A, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(encoder->IRQn_A);

	GPIO_InitStruct.Pin = encoder->GPIO_PIN_B;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	HAL_GPIO_Init(encoder->PORT_PIN_B, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(encoder->IRQn_B, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(encoder->IRQn_B);

	encoder->oldState[0] = HAL_GPIO_ReadPin(encoder->PORT_PIN_A, encoder->PORT_PIN_A);
	encoder->oldState[1] = HAL_GPIO_ReadPin(encoder->PORT_PIN_B, encoder->PORT_PIN_B);
}

void LG_ENCODER_HAS_INTERRUPT(LG_ENCODER * encoder){

	if(__HAL_GPIO_EXTI_GET_IT(encoder->GPIO_PIN_A)){
		LG_ENCODER_COUNTER(encoder);
		__HAL_GPIO_EXTI_CLEAR_IT(encoder->GPIO_PIN_A);
		return;
	}
	if(__HAL_GPIO_EXTI_GET_IT(encoder->GPIO_PIN_B)){
		LG_ENCODER_COUNTER(encoder);
		__HAL_GPIO_EXTI_CLEAR_IT(encoder->GPIO_PIN_B);
		return;
	}
}

void LG_ENCODER_COUNTER(LG_ENCODER * encoder){

	encoder->State[0] = HAL_GPIO_ReadPin(encoder->PORT_PIN_A, encoder->GPIO_PIN_A);
	encoder->State[1] = HAL_GPIO_ReadPin(encoder->PORT_PIN_B, encoder->GPIO_PIN_B);

	if(encoder->State[0] ^ encoder->oldState[1]){
		encoder->count ++;
	}else if(encoder->State[1] ^ encoder->oldState[0]){
		encoder->count --;
	}

	encoder->oldState[0] = encoder->State[0];
	encoder->oldState[1] = encoder->State[1];
}

float LG_ENCODER_GET_POSITION(LG_ENCODER * encoder){
	return encoder->count*encoder->resolution;
}
