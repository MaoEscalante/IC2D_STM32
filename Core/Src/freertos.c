/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#define MM_SIZE 30

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

#include <fc2d_interfaces/msg/fc2d_actuator.h>

#include <stdio.h>
#include <stdarg.h>

#include <math.h>

float getForce(int value){
	return - ((2.225001f)*value + -4703.003558f);
}
static _errorForce = 0.0f;
static _ForceCur = 0.0f;

typedef StaticTask_t osStaticThreadDef_t;
osThreadId_t rosTaskHandle;
uint32_t rosTaskBuffer[ 3000 ];
osStaticThreadDef_t rosTaskControlBlock;
const osThreadAttr_t rosTask_attributes = {
  .name = "rosTask",
  .cb_mem = &rosTaskControlBlock,
  .cb_size = sizeof(rosTaskControlBlock),
  .stack_mem = &rosTaskBuffer[0],
  .stack_size = sizeof(rosTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for sensorTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


void StartROSTask(void *argument);
void StartSensorTask(void *argument);
void ControlSensorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void) {

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  rosTaskHandle = osThreadNew(StartROSTask, NULL, &rosTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  controlTaskHandle = osThreadNew(ControlSensorTask, NULL, &controlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

void StartSensorTask(void *argument)
{

  char text[100];



  BSP_LCD_DisplayStringAt(0, 0, "FC2D - LegRo", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0, 50, "LEFT  [ V ]", LEFT_MODE);
  BSP_LCD_DisplayStringAt(0,175, "RIGTH [ P ]", LEFT_MODE);




  for(;;)
  {
	  if(PA_PRESSED){
			PA_PRESSED = false;
			if(FC2D.STATE+1 == CONTROL_STATES_END) FC2D.STATE = 0;
			else FC2D.STATE ++;

			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
				osDelay(100);
			}
	  }



		sprintf(text," Pos   : %d   ",(int)FC2D.actuator[0].pos);
		BSP_LCD_DisplayStringAt(0, 75, text, LEFT_MODE);
		sprintf(text," Force : %d   ",(int)_ForceCur);
		BSP_LCD_DisplayStringAt(0, 100, text, LEFT_MODE);
		sprintf(text," u     : %d   ",(int)FC2D.actuator[0].u);
		BSP_LCD_DisplayStringAt(0, 125, text, LEFT_MODE);

		sprintf(text," Error: %d ",_errorForce);
		BSP_LCD_DisplayStringAt(0,200, text, LEFT_MODE);
		sprintf(text," Vol: %d ",0);
		BSP_LCD_DisplayStringAt(0,225, text, LEFT_MODE);

		sprintf(text,"STATE: %d ",FC2D.STATE);
		BSP_LCD_DisplayStringAt(0,275, text, LEFT_MODE);


		osDelay(100);
  }

}

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


static char msg_log_t[200];
static rcl_publisher_t pub_stm_log;
static std_msgs__msg__String msg_log;
static void LOG_ROS(const char * format, ...) {
	va_list arg;
	va_start(arg, format);
	vsprintf(msg_log_t,format, arg);
	va_end(arg);
	rcl_ret_t ret = rcl_publish(&pub_stm_log, &msg_log, NULL);
	(void)ret;
}

void StartROSTask(void *argument)
{
	 rmw_uros_set_custom_transport(
		    true,
		    (void *) &huart1,
		    cubemx_transport_open,
		    cubemx_transport_close,
		    cubemx_transport_write,
		    cubemx_transport_read);


		  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
		  freeRTOS_allocator.allocate = microros_allocate;
		  freeRTOS_allocator.deallocate = microros_deallocate;
		  freeRTOS_allocator.reallocate = microros_reallocate;
		  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

		  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		      printf("Error on default allocators (line %d)\n", __LINE__);
		  }

		  printf("Out in Line: %d \n", __LINE__);


		  rclc_support_t support;
		  rcl_allocator_t allocator;
		  rcl_node_t node;
		  allocator = rcl_get_default_allocator();
		  //create init_options
		  rclc_support_init(&support, 0, NULL, &allocator);
		  // create node
		  rclc_node_init_default(&node, "stm32_fc2d", "", &support);

		  msg_log.data.data = (char *)&msg_log_t;
		  rclc_publisher_init_default(
			&pub_stm_log,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
			"stm32_log");


		fc2d_interfaces__msg__Fc2dActuator msg_fc2d_left;

		rcl_publisher_t pub_fc2d;

		rclc_publisher_init_default(
			&pub_fc2d,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(fc2d_interfaces, msg, Fc2dActuator),
			"fc2d");

		msg_fc2d_left.info.data = "LEFT";

		rcl_ret_t ret_fc2d;

		int count  = 0;
		for(;;)
		{
			msg_fc2d_left.pos = LG_ENCODER_GET_POSITION(&FC2D.actuator[0].encoder);
			msg_fc2d_left.u = FC2D.actuator[0].u;
			msg_fc2d_left.force = FC2D.actuator[0].force;

			ret_fc2d = rcl_publish(&pub_fc2d, &msg_fc2d_left, NULL);

			LOG_ROS("Enviado . . . %d",count++);

			osDelay(40);
		}

}





void ControlSensorTask(void *argument){
uint32_t Adc_value=0;
uint64_t count = 0;


float MaxValue = 1000.0;
//float ImoTionZero =4095.0*1.5625f/3.0f;
float ZERO_Ha =4095.0*1.5625f/3.0f;

//Force control parameters:
double force_d = 0.0f;
double force_error = 0.0f;
double error_0 = 0.0f;
double error_1 = 0.0f;
double integral = 0.0f;
double force_control = 0.0f;
double kp = 0.0005; //
double ki = 0.01f; //
double kd = 0.00001f;
double Ts = 1e-3;
double mmean[MM_SIZE] = {0};
int countmm = 0;
double mmean_value = 0.0f;

double force_last = 0;
double force_filtered = 0;
float lpf_cutoff = 5.0;
float lpf_alpha = 2*M_PI*Ts*lpf_cutoff/(2*M_PI*Ts*lpf_cutoff + 1);


float force_zero = getForce(2040);

for(int i = 0 ; i < MM_SIZE ; i++) mmean[i] = 0.0f;

for(;;){



	HAL_ADC_Start(&FC2D_handler.ADC_u);
	if(HAL_ADC_PollForConversion(&FC2D_handler.ADC_u, 5) == HAL_OK) Adc_value  = HAL_ADC_GetValue(&FC2D_handler.ADC_u);
	HAL_ADC_Stop(&FC2D_handler.ADC_u);


	FC2D.actuator[0].pos = FC2D.actuator[0].encoder.count * 1.0f;//LG_ENCODER_GET_POSITION(&FC2D.actuator[0].encoder.count) ;
	FC2D.actuator[0].u = ZERO_Ha;
    //	sinf(2.0f*M_PI*0.5f*(count/1000.0f))*MaxValue + (4096.0/2.0);

	mmean[countmm] = Adc_value;
	countmm ++;
	if(countmm>=MM_SIZE) countmm = 0;
	mmean_value = 0.0f;
	for(int i = 0 ; i < MM_SIZE ; i++) mmean_value += mmean[i];
	mmean_value /= MM_SIZE;


	force_filtered = (lpf_alpha)*Adc_value + (1 - lpf_alpha)*force_last;
	force_last = force_filtered;

	FC2D.actuator[0].force = force_filtered;
	force_d = 0.0f;
	switch(FC2D.STATE){
		case(START):
				FC2D.actuator[0].u = ZERO_Ha;
				count = 0;
					break;
		case(WAIT):
				FC2D.actuator[0].u = ZERO_Ha;
					break;
		case(RUNNING_1):
				force_d = 0.0f ;
					break;
		case(RUNNING_2):
				//FC2D.actuator[0].u   = ZERO_Ha+sinf(2.0f*M_PI*1.0f/10.0f*(count/1000.0f))*MaxValue ;
				force_d = 500.0f ;

				sinf(2.0f*M_PI*1.0f/10.0f*(count/1000.0f))*1000.0f ;
					break;
		case(RUNNING_3):
				force_d = 250.0f ;
						//FC2D.actuator[0].u = ( sinf(2.0f*M_PI*10.0f*(count/1000.0f)) )*MaxValue*.5 + (4096.0/2.0);
					break;
	}


	if(FC2D.STATE == RUNNING_1 || FC2D.STATE == RUNNING_2 || FC2D.STATE == RUNNING_3){
		force_error = force_d - ( getForce(FC2D.actuator[0].force) - force_zero);
		error_0 = force_error;
		integral += error_0*Ts;
		force_control = kp*(error_0)  + ki*Ts*(integral) + kd*(error_0-error_1)/Ts;
		force_control = force_control*4095.0f/3.0f + ZERO_Ha ;


		_errorForce = force_error;
		_ForceCur = ( getForce(FC2D.actuator[0].force) - force_zero);

		if(force_control<=1000) force_control = 100;
		if(force_control>=4000) force_control = 4000;

		FC2D.actuator[0].u = force_control ;

		error_1 = error_0;
	}else{
		FC2D.actuator[0].u = ZERO_Ha ;
	}



	HAL_DAC_SetValue(&DAC_Config, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)FC2D.actuator[0].u);

	count ++;
	osDelay(1);

}
}























