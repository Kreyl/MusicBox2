/*
 * ws2812b.cpp
 *
 *  Created on: 05 апр. 2014 г.
 *      Author: Kreyl
 */

#include "ws2812b.h"

#define LED_DMA_MODE    DMA_PRIORITY_HIGH \
                        | STM32_DMA_CR_MSIZE_HWORD \
                        | STM32_DMA_CR_PSIZE_HWORD \
                        | STM32_DMA_CR_MINC     /* Memory pointer increase */ \
                        | STM32_DMA_CR_DIR_M2P  /* Direction is memory to peripheral */

// Tx timings: bit cnt
#define SEQ_1               0b11111000  // 0xF8
#define SEQ_0               0b11000000  // 0xC0

#define SEQ_00              0x8080
#define SEQ_01              0x80E0
#define SEQ_10              0xE080
#define SEQ_11              0xE0E0

void IntelLeds_t::Init() {
    PinSetupAlterFunc(LEDWS_PIN);
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, sclkDiv2, bitn16);
    ISpi.Enable();
    ISpi.EnableTxDma();

//    Uart.Printf("Led BufSz=%u bytes\r", sizeof(IBuf));

    // Zero buffer
    for(int i=0; i<TOTAL_W_CNT; i++) IBuf[i] = 0;

    // ==== DMA ====
    dmaStreamAllocate     (LEDWS_DMA, IRQ_PRIO_LOW, nullptr, nullptr);
    dmaStreamSetPeripheral(LEDWS_DMA, &LEDWS_SPI->DR);
    dmaStreamSetMode      (LEDWS_DMA, LED_DMA_MODE);
}

void IntelLeds_t::AppendBitsMadeOfByte(uint8_t Byte) {
    uint8_t Bits;
    Bits = Byte & 0b11000000;
    if     (Bits == 0b00000000) *PBuf++ = SEQ_00;
    else if(Bits == 0b01000000) *PBuf++ = SEQ_01;
    else if(Bits == 0b10000000) *PBuf++ = SEQ_10;
    else if(Bits == 0b11000000) *PBuf++ = SEQ_11;

    Bits = Byte & 0b00110000;
    if     (Bits == 0b00000000) *PBuf++ = SEQ_00;
    else if(Bits == 0b00010000) *PBuf++ = SEQ_01;
    else if(Bits == 0b00100000) *PBuf++ = SEQ_10;
    else if(Bits == 0b00110000) *PBuf++ = SEQ_11;

    Bits = Byte & 0b00001100;
    if     (Bits == 0b00000000) *PBuf++ = SEQ_00;
    else if(Bits == 0b00000100) *PBuf++ = SEQ_01;
    else if(Bits == 0b00001000) *PBuf++ = SEQ_10;
    else if(Bits == 0b00001100) *PBuf++ = SEQ_11;

    Bits = Byte & 0b00000011;
    if     (Bits == 0b00000000) *PBuf++ = SEQ_00;
    else if(Bits == 0b00000001) *PBuf++ = SEQ_01;
    else if(Bits == 0b00000010) *PBuf++ = SEQ_10;
    else if(Bits == 0b00000011) *PBuf++ = SEQ_11;

}

void IntelLeds_t::ISetCurrentColors() {
    // Fill bit buffer
    PBuf = IBuf;
    PBuf++;
    for(uint32_t i=0; i<LED_CNT; i++) {
        AppendBitsMadeOfByte(ICurrentClr[i].G);
        AppendBitsMadeOfByte(ICurrentClr[i].R);
        AppendBitsMadeOfByte(ICurrentClr[i].B);
    }
//    for(uint32_t i=0; i<TOTAL_W_CNT; i++)
//        Uart.PrintfNow("\r send n %u data %X ", i, IBuf[i]);
    // Start transmission
    dmaStreamDisable(LEDWS_DMA);
    dmaStreamSetMemory0(LEDWS_DMA, IBuf);
    dmaStreamSetTransactionSize(LEDWS_DMA, TOTAL_W_CNT);
    dmaStreamSetMode(LEDWS_DMA, LED_DMA_MODE);
    dmaStreamEnable(LEDWS_DMA);
}

