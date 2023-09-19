#ifndef __DEFS_LEGRO
#define __DEFS_LEGRO

	#include <main.h>
	#include <stm32f4xx_lg_encoder.h>
	#include <stm32f4xx_lg_dac.h>
	#include <stm32f4xx_lg_adc.h>

	enum{
		START = 0,
		WAIT,
		RUNNING_1,
		RUNNING_2,
		RUNNING_3,
		CONTROL_STATES_END
	};

	typedef struct FC2D_actuator_t{
		LG_ENCODER encoder;
		uint32_t force;
		float u;
		float pos;
	}_FC2D_actuator;

	typedef struct FC2D_handler_t{
		ADC_HandleTypeDef ADC_u;
	}_FC2D_handler;

	typedef struct FC2D_t{
		_FC2D_actuator actuator[2];
		int STATE;
	}_FC2D;

	extern _FC2D FC2D;
	extern _FC2D_handler FC2D_handler;
	extern bool PA_PRESSED ;

#endif
