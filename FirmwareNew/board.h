/*
 * board.h
 *
 *  Created on: 12 сент. 2015 г.
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>

// ==== General ====
#define BOARD_NAME          "MusicBox2_New"
#define APP_NAME            "MusicBox"

// MCU type as defined in the ST header.
#define STM32F205xx

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ 12000000

#define SYS_TIM_CLK     (Clk.APB1FreqHz) // OS timer settings
#define I2C_REQUIRED    FALSE
#define ADC_REQUIRED    TRUE
#define SIMPLESENSORS_ENABLED   TRUE

// Backup Registers
#define TrackNumberBKP    0
#define VolumeBKP         1

// Default Settings
#define DEF_MotorSpeed    -16  // [об/мин * 0,1]
#define DEF_VolLevel      220

#if 1 // ========================== GPIO =======================================

// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     2
#define UART_RX_PIN     3
#define UART_AF         AF7 // for USART2 @ GPIOA

// LED
#define LED_PIN         { GPIOB, 3, TIM2, 2, invNotInverted, omPushPull, 512 }

// Button
#define BUTTONS_CNT     2
#define VolUpIndex      0
#define VolDownIndex    1
#define BTN_VolUp_pin   GPIOA, 6, pudPullUp
#define BTN_VolDown_pin GPIOA, 7, pudPullUp

// Battery Management
#define BattMeasSW_Pin  { GPIOC, 1, omOpenDrain }
#define BattMeas_Pin    GPIOC, 0

// Vibro

// Beeper

// Stepping Motor
#define MotorPins       { GPIOB, 6, 7, 8, 9 }
#define MotorSHDN       4
#define MotorAngle      18
#define MotorRatio      100
// Max. Starting Frequency    900 PPS
// Max. Slewing Frequency     1200 PPS
// Pulse Per Second, т.е импульсов (шагов) за секунду (целых шагов или микрошагов)

// Sensors
#define Sensor1_Pin     GPIOB, 0, pudPullDown
#define Sensor2_Pin     GPIOB, 1, pudPullDown
#define WKUP_pin        { GPIOA, 0, pudPullDown }

// Peripheral power enable
#define PeriphySW_Pin         GPIOC, 14
#define PeriphyPWSW_Pin       GPIOC, 15
#define PeriphySW_PinMode     omOpenDrain

// External Power Input
#define ExternalPWR_Pin    GPIOA, 9, pudPullDown

#endif // GPIO

#if 1 // ========================= Timer =======================================
#endif // Timer

#if I2C_REQUIRED // ====================== I2C =================================
#define I2C1_ENABLED     TRUE
#define I2C_PIN       { GPIOA, 9, 10, I2C1_AF, I2C1_BAUDRATE, I2C1_DMA_TX, I2C1_DMA_RX }
#endif

#if 1 // =========================== SPI =======================================
#endif

#if 1 // ========================== USART ======================================
#define UART            USART2
#define UART_TX_REG     UART->DR
#define UART_RX_REG     UART->DR
#endif

#if ADC_REQUIRED // ======================= Inner ADC ==========================
// Clock divider: clock is generated from the APB2
#define ADC_CLK_DIVIDER		adcDiv4

// ADC channels
#define BAT_CHNL 	        10

//#define ADC_VREFINT_CHNL    17  // All 4xx and F072 devices. Do not change.
#define ADC_CHANNELS        { BAT_CHNL }//{ BAT_CHNL, ADC_VREFINT_CHNL }
#define CallConst           450
#define ADC_CHANNEL_CNT     1   // Do not use countof(AdcChannels) as preprocessor does not know what is countof => cannot check
#define ADC_SAMPLE_TIME     ast239d5Cycles
#define ADC_SAMPLE_CNT      16   // How many times to measure every channel

#define ADC_MAX_SEQ_LEN     16  // 1...16; Const, see ref man
#define ADC_SEQ_LEN         (ADC_SAMPLE_CNT * ADC_CHANNEL_CNT)
#if (ADC_SEQ_LEN > ADC_MAX_SEQ_LEN) || (ADC_SEQ_LEN == 0)
#error "Wrong ADC channel count and sample count"
#endif
#endif

#if 1 // =========================== DMA =======================================
#define STM32_DMA_REQUIRED  TRUE
// ==== Uart ====
// Remap is made automatically if required
#define UART_DMA_TX     STM32_DMA1_STREAM6
#define UART_DMA_RX     STM32_DMA1_STREAM5
#define UART_DMA_CHNL   4

#if I2C_REQUIRED // ==== I2C ====

#endif

#if ADC_REQUIRED
/* DMA request mapped on this DMA channel only if the corresponding remapping bit is cleared in the SYSCFG_CFGR1
 * register. For more details, please refer to Section10.1.1: SYSCFG configuration register 1 (SYSCFG_CFGR1) on
 * page173 */
#define ADC_DMA         STM32_DMA2_STREAM4
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* DMA2 Stream4 Channel0 */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA
