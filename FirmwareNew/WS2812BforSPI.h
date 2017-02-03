/*
 * ws2812b.h
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#pragma once
#if 0
#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "color.h"
#include "uart.h"


#define WS2812_TMR_ENABLED  TRUE

#define LED_CNT         132   // Number of WS2812 LEDs

#define LED_DMA_MODE    DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P  /* Direction is memory to peripheral */ \
                        | STM32_DMA_CR_TCIE     /* Enable Transmission Complete IRQ */

// Tx timings: bit cnt
#define SEQ_1               0b11111000  // 0xF8
#define SEQ_0               0b11000000  // 0xC0

#define SEQ_00              0xC0C0
#define SEQ_01              0xC0F8
#define SEQ_10              0xF8C0
#define SEQ_11              0xF8F8

#define SEQ_LEN             8
#define RST_W_CNT           30 // zero words to produce reset

// SPI16 Buffer (no tuning required)
#define DATA_BIT_CNT        (LED_CNT * 3 * 8 * SEQ_LEN)   // Each led has 3 channels 8 bit each
#define DATA_W_CNT          ((DATA_BIT_CNT + 15) / 16)
#define TOTAL_W_CNT         (RST_W_CNT + DATA_W_CNT + 1)

class LedWs_t {
private:
    Spi_t ISpi;
    uint16_t IBuf[TOTAL_W_CNT];
    uint16_t *PBuf;
    void AppendBitsMadeOfByte(uint8_t Byte);
public:
    void Init();
    // Inner use
    Color_t ICurrentClr[LED_CNT];
    void ISetCurrentColors();
    void ITmrHandlerI();
};

extern LedWs_t LedWs;

#endif
