/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  * 		-- Kevin Dong modified on 2022-05-18
  * 			serve as base project for ENGG4420 & ENGG6550
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdarg.h>

/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "spi.h"
#include "i2c.h"
#include "camera.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osMessageQId myQueue01Handle;
osMutexId myMutex01Handle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId camSemHandle;


/* USER CODE BEGIN Variables */
uint8_t RxData[256];
uint32_t data_received;
uint8_t Str4Display[50];
osThreadId TaskGUIHandle;
osThreadId TaskCOMMHandle;
osThreadId TaskBTNHandle;
osThreadId TaskSNSRHandle;
osThreadId TaskCameraHandle;
osMessageQId CommQueueHandle;
osMutexId dataMutexHandle;
osSemaphoreId printSemHandle;
osSemaphoreId lcdSemHandle;

uint8_t sensor_flag;


typedef struct
{
	uint8_t Value[10];
	uint8_t Source;
}data;

data DataToSend={"Hello\0", 1};
data DataVCP={"VCP\0",2};

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
int myprintf(const char *format, ...);

/* USER CODE BEGIN FunctionPrototypes */
void StartTaskCOMM(void const * argument);
void StartTaskBTN(void const * argument);
void StartTaskGUI(void const * argument);
void StartTaskCamera(void const * argument);
void SensorTask(void const * argument);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {

	osMutexDef(dataMutex);
	dataMutexHandle = osMutexCreate(osMutex(dataMutex));

	/* definition and creation of printSem */
	osSemaphoreDef(printSem);
	printSemHandle = osSemaphoreCreate(osSemaphore(printSem), 1);

	/* definition and creation of lcdSem */
	osSemaphoreDef(lcdSem);
	lcdSemHandle = osSemaphoreCreate(osSemaphore(lcdSem), 1);

	osThreadDef(TaskCOMM, StartTaskCOMM, osPriorityHigh, 0, 128);
	TaskCOMMHandle = osThreadCreate(osThread(TaskCOMM), NULL);

	osThreadDef(TaskBTN, StartTaskBTN, osPriorityLow, 0, 128);
	TaskBTNHandle = osThreadCreate(osThread(TaskBTN), NULL);

	osThreadDef(TaskGUI, StartTaskGUI, osPriorityNormal, 0, 128);
	TaskGUIHandle = osThreadCreate(osThread(TaskGUI), NULL);

	osThreadDef(TaskSNSR, SensorTask, osPriorityNormal, 0, 128);
	TaskSNSRHandle = osThreadCreate(osThread(TaskSNSR), NULL);

	osThreadDef(TaskCamera, StartTaskCamera, osPriorityNormal, 0, 1024);
	TaskCameraHandle = osThreadCreate(osThread(TaskCamera), NULL);

	osMessageQDef(myQueue01, 1, data);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

	/* Comm QUEUE, don't delete */
	osMessageQDef(CommQueue, 1, &DataVCP);
	CommQueueHandle = osMessageCreate(osMessageQ(CommQueue), NULL);
}


void StartTaskCOMM(void const * argument)
{

  osEvent vcpValue;

  while(1)
  {
	  vcpValue = osMessageGet(CommQueueHandle, osWaitForever);
	  osMutexWait(dataMutexHandle, 0);
	  memcpy(Str4Display,(char *)(((data *)vcpValue.value.p)->Value), data_received+1);
	  osMutexRelease(dataMutexHandle);
	  myprintf("CommTask received: %s\n\r", Str4Display);

	  // if received 'c', capture a picture from camera
	  if (Str4Display[0] == 'c')
	  {
		  myprintf("hello world\n\r");
	  }
  }
}

void StartTaskGUI(void const * argument)
{
  uint8_t buf[sizeof(Str4Display)];


  while(1)
  {
	  osSemaphoreWait(lcdSemHandle, osWaitForever);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)" Security Camera ");
	  if(Str4Display[0]!='\0')
	  {
		  osMutexWait(dataMutexHandle, 0);
		  memcpy(buf,Str4Display, data_received+1);
		  Str4Display[0] = '\0';
		  osMutexRelease(dataMutexHandle);
		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_ClearStringLine(7);
		  BSP_LCD_DisplayStringAtLine(7, buf);
		  printf("GUI task received: %s\n\r", buf);
	  }
	  osSemaphoreRelease(lcdSemHandle);

	  osDelay(500);
  }
}

void StartTaskBTN(void const * argument)
{
  uint8_t count_press = 0;

  while(1)
  {
	  if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){
		  myprintf("TaskBTN Button Pressed : %i times \n\r", count_press);
		  count_press++;
		  while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
	  }
	  osDelay(10);
  }
}

void StartTaskCamera(void const * argument)
{

	// setup camera
	camera_setup();
	osSemaphoreWait(camSemHandle, osWaitForever);

	while (1) {

		osSemaphoreWait(lcdSemHandle, osWaitForever);
		camera_initiate_capture();
		osSemaphoreRelease(lcdSemHandle);
		//printf("Camera Semaphore released \n \r");

		osDelay(100);
	}
}

void SensorTask(void const * argument)
{
	osSemaphoreWait(camSemHandle, osWaitForever);
	while(1){
		printf("Scanning...\n\r");
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){
			printf("\n ALERT: INTRUDER DETECTED!\n\r");
			osSemaphoreRelease(camSemHandle);
			while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET);
		}
		osDelay(500);
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET){
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
		sensor_flag = 1;
	}
}


int myprintf(const char *format, ...)
{
	osSemaphoreWait(printSemHandle, osWaitForever);
	va_list alist;
	va_start(alist, format);
	vprintf(format, alist);
    va_end(alist);
	osSemaphoreRelease(printSemHandle);
	return 0;
}
