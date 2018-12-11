/*
 * nrf.c
 *
 *  Created on: Dec 10, 2018
 *      Author: yliu60
 */

#include <stm32f4xx_hal.h>
#include <stdint.h>
#include "spi.h"

#define R_REGISTER              0x00
#define W_REGISTER              0x20
#define R_RX_PAYLOAD            0b01100001
#define W_TX_PAYLOAD            0b10100000
#define FLUSH_TX                0b11100001
#define FLUSH_RX                0b11100010
#define REUSE_TX_PL             0b11100011
#define R_RX_PL_WID             0b01100000
#define W_ACK_PAYLOAD           0b10101000
#define W_TX_PAYLOAD_NO_ACK     0b10110000
#define NOP                     0xFF
#define ACTIVATE                0b01010000

#define CONFIG                  0x00
#define EN_AA                   0x01
#define EN_RXADDR               0x02
#define SETUP_AW                0x03
#define SETUP_RETR              0x04
#define RF_CH                   0x05
#define RF_SETUP                0x06
#define STATUS                  0x07
#define OBSERVE_TX              0x08
#define RPD                     0x09
#define RX_ADDR_P0              0x0A
#define RX_ADDR_P1              0x0B
#define RX_ADDR_P2              0x0C
#define RX_ADDR_P3              0x0D
#define RX_ADDR_P4              0x0E
#define RX_ADDR_P5              0x0F
#define TX_ADDR                 0x10
#define RX_PW_P0                0x11
#define RX_PW_P1                0x12
#define RX_PW_P2                0x13
#define RX_PW_P3                0x14
#define RX_PW_P4                0x15
#define RX_PW_P5                0x16
#define FIFO_STATUS             0x17
#define DYNPD                   0x1C
#define FEATURE                 0x1D


static void sendCommandNoValue(uint8_t command)
{
	SPI_csLow();
	SPI_transfer(command);
	SPI_csHigh();
}

static void writeRegister(uint8_t reg, uint8_t val)
{
	SPI_csLow();
	SPI_transfer(W_REGISTER | reg);
	SPI_transfer(val);
	SPI_csHigh();
}

static void writeRegisterArray(uint8_t reg, uint8_t val[], uint8_t len)
{
	SPI_csLow();
	SPI_transfer(W_REGISTER | reg);
	int i;
	for (i = 0; i < len; i++){
		SPI_transfer(val[i]);
	}
	SPI_csHigh();
}

//PC4 = CE, PC5 = CS
static void ceHigh()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

static void ceLow()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

//static uint8_t readRegister(uint8_t reg)
//{
//    SPI_csLow();
//    SPI_transfer(R_REGISTER | reg);
//    uint8_t val = SPI_transfer(0xFF);
//    SPI_csHigh();
//    return val;
//}

void NRF_init()
{
	uint8_t RX_address[] = {0x0B, 0xC2, 0xC2};      // LSByte first
	uint8_t TX_address[] = {0x0B, 0xC2, 0xC2};      // LSByte first
	SPI_init();
	writeRegister(ACTIVATE, 0x73);
	writeRegister(CONFIG, 0b00110010);              // RX_DR interrupt on IRQ active low; CRC 1 byte; power up; PTX
	writeRegister(EN_AA, 0b00000001);               // enable auto ack data pipe 0
	writeRegister(EN_RXADDR, 0b00000001);           // enable data pipe 0
	writeRegister(SETUP_AW, 0b00000001);            // address width = 3 bytes
	writeRegister(RF_CH, 0b00001001);               // channel 9
	writeRegister(RF_SETUP, 0b00100110);            // 256 kbps data rate
	writeRegisterArray(RX_ADDR_P0, RX_address, 3);  // RX address on pipe 0 = 0xC2C20B
	writeRegisterArray(TX_ADDR, TX_address, 3);     // TX address = 0xC2C20B
	writeRegister(FEATURE, 0b00000100);             // enable dynamic payload length; enable payload with ACK
	writeRegister(DYNPD, 0x01);                     // enable dynamic payload length data pipe 0; requires EN_DPL and ENAA first
	writeRegister(STATUS, 0b01111110);              // write 1 to MAX_RT to clear it
	ceLow();
}

void NRF_transmit(uint8_t data[], uint8_t len)
{
	// flush TX FIFO
	sendCommandNoValue(FLUSH_TX);
	// clear status flags
	writeRegister(STATUS, 0b01111110);
	// write payload
	uint8_t i;
	SPI_csLow();
	SPI_transfer(W_TX_PAYLOAD);
	for (i = 0; i < len; i++)
	{
		SPI_transfer(data[i]);
	}
	SPI_csHigh();

	// ce pulse for ~10us
	ceHigh();
	for (int i = 0; i<255; i++){}
	ceLow();

	HAL_Delay(1);
}
