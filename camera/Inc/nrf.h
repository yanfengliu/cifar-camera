/*
 * nrf.h
 *
 *  Created on: Dec 10, 2018
 *      Author: yliu60
 */

#pragma once
#include <stdint.h>


void NRF_init();
void NRF_transmit(uint8_t data[], uint8_t len);
