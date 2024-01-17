/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"

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
/* USER CODE BEGIN Variables */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END Variables */
/* Definitions for LEDTASK */
osThreadId_t LEDTASKHandle;
const osThreadAttr_t LEDTASK_attributes = {
  .name = "LEDTASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KeyBoardTask */
osThreadId_t KeyBoardTaskHandle;
const osThreadAttr_t KeyBoardTask_attributes = {
  .name = "KeyBoardTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ButtonTask */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for buttonTrigger */
osSemaphoreId_t buttonTriggerHandle;
const osSemaphoreAttr_t buttonTrigger_attributes = {
  .name = "buttonTrigger"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LedFun(void *argument);
void KeyboardFun(void *argument);
void ButtonFun(void *argument);
void ADCFun(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of buttonTrigger */
  buttonTriggerHandle = osSemaphoreNew(1, 0, &buttonTrigger_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDTASK */
  LEDTASKHandle = osThreadNew(LedFun, NULL, &LEDTASK_attributes);

  /* creation of KeyBoardTask */
  KeyBoardTaskHandle = osThreadNew(KeyboardFun, NULL, &KeyBoardTask_attributes);

  /* creation of ButtonTask */
  ButtonTaskHandle = osThreadNew(ButtonFun, NULL, &ButtonTask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(ADCFun, NULL, &ADCTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LedFun */
/**
 * @brief  Function implementing the LEDTASK thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_LedFun */
void LedFun(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN LedFun */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		osDelay(200);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		osDelay(200);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		osDelay(200);
	}
  /* USER CODE END LedFun */
}

/* USER CODE BEGIN Header_KeyboardFun */
/**
 * @brief Function implementing the KeyBoardTask thread.
 * @param argument: Not used
 * @retval None
 */

/*
 * STEP TO DO WHEN WE REBUILD code:
 *  1. in file Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usdb_hid.c:
 *     - in line 161 change nInterfaceProtocol to 1 (keyboard) /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
 *     - from URL https://github.com/NordicSemiconductor/ble-sdk-arduino/blob/master/libraries/BLE/examples/ble_HID_keyboard_template/USD%20HID%20Report%20Descriptor%20-%20Keyboard.txt
 *        copy content of the file (start in line 3) and replace it in HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] in line 220 (whole array)
 *  2. in file Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usdb_hid.d:
 *     - replace HID_MOUSE_REPORT_DESC_SIZE property with 63U  (by default it should be 74U)
 *  3. in file https://www.usb.org/sites/default/files/documents/hid1_11.pdf we have definition of "keyboard" protocol on page 70
 *  4. in file https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf we have "key codes" on page 53
 */

/* USER CODE END Header_KeyboardFun */
void KeyboardFun(void *argument)
{
  /* USER CODE BEGIN KeyboardFun */
	osSemaphoreAcquire(buttonTriggerHandle, osWaitForever);
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* Infinite loop */
	uint8_t hidBuffer[8] = {0};
	for(;;)
	{
		if (osSemaphoreAcquire(buttonTriggerHandle, 200) == osOK)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

			hidBuffer[2] = 0x2C; //spacebar click
			USBD_HID_SendReport(&hUsbDeviceFS, hidBuffer, 8);
			HAL_Delay(50);

			hidBuffer[2] = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, hidBuffer, 8);
			HAL_Delay(50);
		}
	}
  /* USER CODE END KeyboardFun */
}

/* USER CODE BEGIN Header_ButtonFun */
/**
 * @brief Function implementing the ButtonTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ButtonFun */
void ButtonFun(void *argument)
{
  /* USER CODE BEGIN ButtonFun */
	/* Infinite loop */
	uint8_t previousState = 0;
	uint8_t currentState = 0;
	for(;;)
	{
		currentState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

		if(currentState == GPIO_PIN_SET && currentState != previousState)
			osSemaphoreRelease(buttonTriggerHandle);

		previousState = currentState;
		osDelay(30);
	}
  /* USER CODE END ButtonFun */
}

/* USER CODE BEGIN Header_ADCFun */
extern ADC_HandleTypeDef hadc1;
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADCFun */
void ADCFun(void *argument)
{
  /* USER CODE BEGIN ADCFun */
  /* Infinite loop */
  const uint8_t size = 2;
  const uint8_t MA_SIZE = 1 << size; //8 samples

  uint8_t index = 0;
  uint32_t sum = 0;

  for(;;)
  {
	  if(HAL_ADC_Start(&hadc1) == HAL_OK)
	  {
		  if(HAL_ADC_PollForConversion(&hadc1, 200) == HAL_OK)
		  {
			  sum += HAL_ADC_GetValue(&hadc1);
			  index++;

			  if(index >= MA_SIZE){
				  uint32_t avearage = sum >> size; //calculate average of last MA_SIZE values
				  if (avg > 500)
					  osSemaphoreRelease(buttonTriggerHandle);
				  index = 0;
				  sum = 0;
			  }

		  }
		  HAL_ADC_Stop(&hadc1);
	  }
	  osDelay(20);
  }
  /* USER CODE END ADCFun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

