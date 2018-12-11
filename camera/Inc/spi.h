/*
 * spi.h
 *
 *  Created on: Dec 10, 2018
 *      Author: yliu60
 */

#pragma once
#include <stdint.h>

void SPI_init();
void SPI_csLow();
void SPI_csHigh();
void SPI_transfer(uint8_t value);
