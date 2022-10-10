/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "wii_accessory.h"
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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	/* Implement your write code here, this is used by puts and printf for example */
	for (int i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_Delay(100);
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();
	HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t errcode;
	Wii_Accessory_Data wii_accessory_data;
	USB_JoystickReport_Data_t joystick_data;

	while (1) {
		errcode = wii_accessory_poll(&hi2c1, &wii_accessory_data);
		if (errcode == HAL_OK) {
#ifdef DEBUG
			printf("Device type: %d, Data format: %d\r\n",
					wii_accessory_data.device_type,
					wii_accessory_data.data_format);
			printf(
					"jx=%d, jy=%d, rx=%d, ry=%d, tl=%d, tr=%d, acc_x=%d, acc_y=%d, acc_z=%d\r\n",
					wii_accessory_data.jx, wii_accessory_data.jy,
					wii_accessory_data.rx, wii_accessory_data.ry,
					wii_accessory_data.tl, wii_accessory_data.tr,
					wii_accessory_data.acc_x, wii_accessory_data.acc_y,
					wii_accessory_data.acc_z);
			printf(
					"left=%u, right=%u, up=%u, down=%u, a=%u, b=%u, x=%u, y=%u, select=%u, start=%u, home=%u, l=%u, r=%u, zl=%u, zr=%u, wtf=%u\r\n",
					wii_accessory_data.dpad_left, wii_accessory_data.dpad_right,
					wii_accessory_data.dpad_up, wii_accessory_data.dpad_down,
					wii_accessory_data.button_a, wii_accessory_data.button_b,
					wii_accessory_data.button_x, wii_accessory_data.button_y,
					wii_accessory_data.button_select,
					wii_accessory_data.button_start,
					wii_accessory_data.button_home, wii_accessory_data.button_l,
					wii_accessory_data.button_r, wii_accessory_data.button_zl,
					wii_accessory_data.button_zr, wii_accessory_data.wtf);
#endif
			memset(&joystick_data, 0, sizeof(joystick_data));
			joystick_data.LX = 0x80 + wii_accessory_data.jx;
			joystick_data.LY = 0x80 + wii_accessory_data.jy;
			if (wii_accessory_data.data_format == 0)
			{
				// Nunchuck
				// Accelerometers
				joystick_data.RX = 0x80 + wii_accessory_data.acc_x;
				joystick_data.RY = 0x80 + wii_accessory_data.acc_y;
				//joystick_data.ExtraX = 0x80 + wii_accessory_data.acc_z;
			} else {
				joystick_data.RX = 0x80 + wii_accessory_data.rx;
				joystick_data.RY = 0x80 + wii_accessory_data.ry;
				// Unsigned for trigger analog buttons
				//joystick_data.ExtraX = wii_accessory_data.tl;
				//joystick_data.ExtraY = wii_accessory_data.tr;
			}
			joystick_data.A = wii_accessory_data.button_a;
			joystick_data.B = wii_accessory_data.button_b;
			joystick_data.X = wii_accessory_data.button_x;
			joystick_data.Y = wii_accessory_data.button_y;
			joystick_data.L = wii_accessory_data.button_l;
			joystick_data.R = wii_accessory_data.button_r;
			joystick_data.ZL = wii_accessory_data.button_zl;
			joystick_data.ZR = wii_accessory_data.button_zr;
			joystick_data.minus = wii_accessory_data.button_select;
			joystick_data.plus = wii_accessory_data.button_start;

			if (wii_accessory_data.dpad_up && wii_accessory_data.dpad_right)
				joystick_data.hat_switch = 1;
			else if (wii_accessory_data.dpad_right && wii_accessory_data.dpad_down)
				joystick_data.hat_switch = 3;
			else if (wii_accessory_data.dpad_down && wii_accessory_data.dpad_left)
				joystick_data.hat_switch = 5;
			else if (wii_accessory_data.dpad_left && wii_accessory_data.dpad_up)
				joystick_data.hat_switch = 7;
			else if (wii_accessory_data.dpad_up)
				joystick_data.hat_switch = 0;
			else if (wii_accessory_data.dpad_right)
				joystick_data.hat_switch = 2;
			else if (wii_accessory_data.dpad_down)
				joystick_data.hat_switch = 4;
			else if (wii_accessory_data.dpad_left)
				joystick_data.hat_switch = 6;
			else
				joystick_data.hat_switch = 8;
		} else {
			memset(&joystick_data, 0, sizeof(joystick_data));
		}

		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (void*) &joystick_data,
				sizeof(joystick_data));

		switch (errcode) {
		case WII_ACCESSORY_OK:
			HAL_GPIO_WritePin(LED_WII_GPIO_Port, LED_WII_Pin, GPIO_PIN_RESET);
			break;
		case WII_ACCESSORY_NOT_CONNECTED:
			HAL_GPIO_WritePin(LED_WII_GPIO_Port, LED_WII_Pin, GPIO_PIN_SET);
			break;
		default:
			// Blink on error
			HAL_GPIO_WritePin(
			LED_WII_GPIO_Port, LED_WII_Pin,
					(HAL_GetTick() % 100 < 50) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			break;
		}

		switch (hUsbDeviceFS.dev_state) {
		case USBD_STATE_CONFIGURED:
			HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
			break;
		default:
			// Blink on error
			HAL_GPIO_WritePin(
			LED_USB_GPIO_Port, LED_USB_Pin,
					(HAL_GetTick() % 1000 > 100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			break;
		}

		HAL_Delay(1);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_USB_Pin|LED_WII_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_USB_Pin LED_WII_Pin */
  GPIO_InitStruct.Pin = LED_USB_Pin|LED_WII_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DTCT_Pin */
  GPIO_InitStruct.Pin = DTCT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DTCT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
