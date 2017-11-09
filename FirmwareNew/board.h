/*
 * board.h
 *
 *  Created on: 12 сент. 2015 г.
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>

// ==== General ====
#define BOARD_NAME      "MusicBox2_v2.2"
//#define MusicBox
#define Phone

// Default Settings
#if defined MusicBox
#define APP_NAME        "MusicBox"
#define OFF_delay_MS    700
#elif defined Phone
#define APP_NAME        "Phone"
#define OFF_delay_MS    3000
#define Dir_03          "03"
#define Dir_Any         "Any"
#define BeepTrack       "Sys/beep_32.mp3"   // длинный гудок
#define BusyTrack       "Sys/busy.mp3"      // "занято"
#define WaitTrack       "Sys/wait.mp3"      // "ожидание"
#define minWait_MS      3000
#define maxWait_MS      10000
#endif
// Saund
#define PlayDir         "0:\\"
#define DEF_VolLevel    170
// Motor
#define DEF_MotorSpeed  -600  // 16 [об/мин * 0,1]
// RGB LEDs
#define DEF_LEDsProf    prColor
#define LED_CNT         15  // Number of WS2812 LEDs
#define StartIntensity      6
#define StartProcessTime    200
#define StartPause          0
#define StopIntensity       0
#define StopProcessTime     100
#define StopPause           0
#define DEF_Level_R     2
#define DEF_Level_G     60
#define DEF_Level_B     100
#define DEF_Limit_MAX   100
#define DEF_Limit_MIN   4

// Backup Registers
#define TrackNumberBKP  0
#define VolumeBKP       1

// MCU type as defined in the ST header.
#define STM32F205xx

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ         12000000

#define SYS_TIM_CLK             (Clk.APB1FreqHz) // OS timer settings
#define I2C_REQUIRED            FALSE
#define ADC_REQUIRED            TRUE
#define SIMPLESENSORS_ENABLED   TRUE

#if 1 // ========================== GPIO =======================================

// Button
#define BUTTONS_CNT     3
#define VolUpIndex      0
#define VolDownIndex    1
#define UserIndex       2
#define BTN_VolUp_pin   GPIOA, 7, pudPullUp
#define BTN_VolDown_pin GPIOA, 6, pudPullUp
#define BTN_User_pin    GPIOA, 5, pudPullUp

// Peripheral power enable
#define PeriphySW_Pin         GPIOC, 14
#define PeriphyPWSW_Pin       GPIOC, 15
#define PeriphySW_PinMode     omOpenDrain

// External Power Input
#define ExternalPWR_Pin    GPIOA, 9, pudPullDown

// Battery Management
#define BattMeasSW_Pin  { GPIOC, 1, omOpenDrain }
#define BattMeas_Pin    GPIOC, 0

// Vibro

// Beeper

// Sensors
#define Sensor1_Pin     GPIOB, 0, pudPullDown
#define Sensor2_Pin     GPIOB, 1, pudPullDown
#define WKUP_pin        { GPIOA, 0, pudPullDown }

// LED
#define LED_PIN         { GPIOB, 3, TIM2, 2, invNotInverted, omPushPull, 512 }

// WS2812
#define LEDWS_PIN       GPIOB, 5, omPushPull, pudNone, AF6

// Stepping Motor
#define MotorPins       { GPIOB, 6, 7, 8, 9 }
#define MotorSHDN       4
#define MotorAngle      18
#define MotorRatio      1   //100
// Max. Starting Frequency    900 PPS
// Max. Slewing Frequency     1200 PPS
// Pulse Per Second, т.е импульсов (шагов) за секунду (целых шагов или микрошагов)

// Rotary Dialer
#define Dial_Namber_GPIO    GPIOA
#define Dial_Namber_PIN     1
#define Dial_Disk_GPIO      GPIOB
#define Dial_Disk_PIN       1

// ==== Sound VS1011 ====
#define VS_GPIO         GPIOB
// Pins
#define VS_XCS          10
#define VS_XDCS         11
#define VS_RST          12
#define VS_DREQ         2
#define VS_XCLK         13
#define VS_SO           14
#define VS_SI           15
// Amplifier
#define VS_AMPF_EXISTS  TRUE
#define VS_AMPF_GPIO    GPIOA
#define VS_AMPF_PIN     15

// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     2
#define UART_RX_PIN     3
#define UART_AF         AF7 // for USART2 @ GPIOA

#endif // GPIO

#if 1 // ========================= Timer =======================================
#endif // Timer

#if I2C_REQUIRED // ====================== I2C =================================
#define I2C1_ENABLED     TRUE
#define I2C_PIN       { GPIOA, 9, 10, I2C1_AF, I2C1_BAUDRATE, I2C1_DMA_TX, I2C1_DMA_RX }
#endif

#if 1 // =========================== SPI =======================================
#define VS_SPI          SPI2
#define VS_AF           AF5
#define LEDWS_SPI       SPI3
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
//STM32F205 Reference Manual s.179,180
#define STM32_DMA_REQUIRED  TRUE
// ==== Uart ====
// Remap is made automatically if required
#define UART_DMA_TX     STM32_DMA1_STREAM6
#define UART_DMA_RX     STM32_DMA1_STREAM5
#define UART_DMA_CHNL   4

#if I2C_REQUIRED // ==== I2C ====
#endif

// ==== SPI2 ==== ==== Sound VS1011 ====
#define VS_DMA          STM32_DMA1_STREAM4 // SPI2_TX
#define VS_DMA_CHNL     0   // Dummy
#define VS_DMA_MODE     STM32_DMA_CR_CHSEL(VS_DMA_CHNL) | \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_DIR_M2P |    /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */

// ==== SPI3 ====
#define LEDWS_DMA       STM32_DMA1_STREAM7
#define LEDWS_DMA_CHNL  0   // Dummy

#if ADC_REQUIRED
/* DMA request mapped on this DMA channel only if the corresponding remapping bit is cleared in the SYSCFG_CFGR1
 * register. For more details, please refer to Section10.1.1: SYSCFG configuration register 1 (SYSCFG_CFGR1) on
 * page173 */
#define ADC_DMA         STM32_DMA2_STREAM4
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* DMA2 Stream4 Channel 0 */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA
