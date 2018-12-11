/*
 * spi.c
 *
 *  Created on: Dec 10, 2018
 *      Author: yliu60
 */

#include <stm32f4xx_hal.h>
#include "spi.h"

//PC4 = CE, PC5 = CS

void SPI_init(){
	// set CS high
	SPI_csHigh();
}


void SPI_csLow(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}


void SPI_csHigh(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
}

void SPI_transfer(uint8_t value){
	extern SPI_HandleTypeDef hspi1;
	HAL_SPI_Transmit(&hspi1, &value, 1, 100);
}
