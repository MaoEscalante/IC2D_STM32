#ifndef __ENCODER_LEGRO
#define __ENCODER_LEGRO

/*https://www.rls.si/eng/fileuploader/download/download/?d=1&file=custom%2Fupload%2FLM10D01_15EN_datasheet.pdf*/

typedef enum {
	Increment =  1, /*!< Encoder was incremented */
	Decrement = -1, /*!< Encoder was decremented */
} encRotation_t;

typedef struct lg_encoder_t {
    float	        position;       /*!< Encoder position */
	float			zeroPosition;   /*!< Zero position */
	bool			oldState[2];
	bool			State[2];
	float   		resolution;     /*!< Encoder resolution */
	encRotation_t 	rotation; 		/*!< Rotation direction: Increment, Decrement or nothing */
	int32_t 		count;	        /*!< Number of counts from timer */
	int32_t 		previousCount;	/*!< Previous number of counts from timer */
	int32_t 		countDiff;		/*!< Difference between previous and current counts */
    GPIO_TypeDef* 	PORT_PIN_A;		/*!< Pointer to GPIOx for Rotary encode A pin. Meant for private use */
	GPIO_TypeDef* 	PORT_PIN_B;		/*!< Pointer to GPIOx for Rotary encode B pin. Meant for private use */
	uint16_t 		GPIO_PIN_A;     /*!< GPIO pin for rotary encoder A pin. This pin is also set for interrupt */
	uint16_t 		GPIO_PIN_B;     /*!< GPIO pin for rotary encoder B pin. */
	IRQn_Type       IRQn_A;
	IRQn_Type       IRQn_B;
} LG_ENCODER;

void  LG_INIT_ENCODER(LG_ENCODER * encoder);
void  LG_ENCODER_HAS_INTERRUPT(LG_ENCODER * encoder);
float LG_ENCODER_GET_POSITION(LG_ENCODER * encoder);

#endif
