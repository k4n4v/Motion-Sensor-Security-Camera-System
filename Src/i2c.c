/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "spi.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c3;

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */
  
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    GPIO_InitStruct.Pin = I2C3_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SDA_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C3_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SCL_GPIO_Port, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
  
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    HAL_GPIO_DeInit(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin);

    HAL_GPIO_DeInit(I2C3_SCL_GPIO_Port, I2C3_SCL_Pin);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
 * Helper method to read from an 8-bit address over I2C.
 */
Devices_StatusTypeDef
i2c_read8(uint8_t device, uint8_t address, uint8_t* data, uint8_t length) {
	/* Indicate the start address of the read. */
	if (HAL_I2C_Master_Transmit(&hi2c3, device, &address, 1, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}

	/* Read the desired number of bytes. */
	if (HAL_I2C_Master_Receive(&hi2c3, device, data, length, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}

	return DEVICES_OK;
}

/**
 * Helper method to read from a 16-bit address over I2C.
 */
Devices_StatusTypeDef
i2c_read16(uint8_t device, uint16_t address, uint8_t* data, uint8_t length) {
	/* Indicate the start address of the read. */
	//address = ((address & 0xFF00) >> 8) | ((address & 0xFF) << 8);
	address = __builtin_bswap16(address);
	if (HAL_I2C_Master_Transmit(&hi2c3, device, (uint8_t*)&address, 2, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}

	/* Read the desired number of bytes. */
	if (HAL_I2C_Master_Receive(&hi2c3, device, data, length, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}

	return DEVICES_OK;
}

/**
 * Helper method to write an 8-bit frame over I2C.
 */
Devices_StatusTypeDef
i2c_read_frame(uint8_t device, uint8_t *data, uint8_t length){
	/* Read the desired number of bytes. */
	if (HAL_I2C_Master_Receive(&hi2c3, device, data, length, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}

	return DEVICES_OK;
}

/**
 * Helper method to write an 8-bit value to a 8-bit address over I2C.
 */
Devices_StatusTypeDef
i2c_write8_8(uint8_t device, uint8_t address, uint8_t data) {
	/* Write the address and data. */
	uint8_t buffer[2] = { address, data };
	if (HAL_I2C_Master_Transmit(&hi2c3, device, buffer, 2, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}
	return DEVICES_OK;
}

/**
 * Helper method to write an 8-bit value to a 16-bit address over I2C.
 */
Devices_StatusTypeDef
i2c_write16_8(uint8_t device, uint16_t address, uint8_t data) {
	/* Write the address and data. */
	uint8_t output[3] = { (address & 0xFF00) >> 8, (address & 0xFF), data };
	if (HAL_I2C_Master_Transmit(&hi2c3, device, output, 3, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}
	return DEVICES_OK;
}

/**
 * Helper method to write an 8-bit frame over I2C.
 */
Devices_StatusTypeDef
i2c_write_frame(uint8_t device, uint8_t *data, uint8_t length){
	if (HAL_I2C_Master_Transmit(&hi2c3, device, data, length, 1000) != HAL_OK) {
		return DEVICES_ERROR;
	}
	return DEVICES_OK;
}

/**
 * Helper method to write an array of 8-bit values to 16-bit addresses.
 */
Devices_StatusTypeDef
i2c_array16_8(uint8_t device, RegisterTuple16_8* array) {
	while(array->address != 0xFFFF) {
		if (i2c_write16_8(device, array->address, array->value) != DEVICES_OK) {
			return DEVICES_ERROR;
		}
		++array;
	}
	return DEVICES_OK;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
