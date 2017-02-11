/*
 * kl_lib_f0.cpp
 *
 *  Created on: 10.12.2012
 *      Author: kreyl
 */

#include "kl_lib_f2xx.h"
#include "uart.h"
#include <stdarg.h>
#include <string.h>

#if 1 // ============================= Timer ===================================
void Timer_t::Init() const {
#if defined STM32L1XX
    if     (ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
    else if(ITmr == TIM6)  { rccEnableAPB1(RCC_APB1ENR_TIM6EN,  FALSE); }
    else if(ITmr == TIM7)  { rccEnableAPB1(RCC_APB1ENR_TIM7EN,  FALSE); }
    else if(ITmr == TIM9)  { rccEnableAPB2(RCC_APB2ENR_TIM9EN,  FALSE); }
    else if(ITmr == TIM10) { rccEnableAPB2(RCC_APB2ENR_TIM10EN, FALSE); }
    else if(ITmr == TIM11) { rccEnableAPB2(RCC_APB2ENR_TIM11EN, FALSE); }
#elif defined STM32F0XX
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
#ifdef TIM2
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
#endif
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
#ifdef TIM6
    else if(ITmr == TIM6)  { rccEnableAPB1(RCC_APB1ENR_TIM6EN,  FALSE); }
#endif
    else if(ITmr == TIM14) { RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; }
#ifdef TIM15
    else if(ITmr == TIM15) { RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; }
#endif
    else if(ITmr == TIM16) { RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; }
    else if(ITmr == TIM17) { RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; }
#elif defined STM32F2XX || defined STM32F4XX
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
    else if(ITmr == TIM5)  { rccEnableTIM5(FALSE); }
    else if(ITmr == TIM6)  { rccEnableTIM6(FALSE); }
    else if(ITmr == TIM7)  { rccEnableTIM7(FALSE); }
    else if(ITmr == TIM8)  { rccEnableTIM8(FALSE); }
    else if(ITmr == TIM9)  { rccEnableTIM9(FALSE); }
    else if(ITmr == TIM10)  { RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; }
    else if(ITmr == TIM11)  { rccEnableTIM11(FALSE); }
    else if(ITmr == TIM12)  { rccEnableTIM12(FALSE); }
    else if(ITmr == TIM13)  { RCC->APB1ENR |= RCC_APB1ENR_TIM13EN; }
    else if(ITmr == TIM14)  { rccEnableTIM14(FALSE); }
#elif defined STM32F10X_LD_VL
    if(ANY_OF_4(ITmr, TIM1, TIM15, TIM16, TIM17)) PClk = &Clk.APB2FreqHz;
    else PClk = &Clk.APB1FreqHz;
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM15) { RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; }
    else if(ITmr == TIM16) { RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; }
    else if(ITmr == TIM17) { RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; }
#elif defined STM32L4XX
    if     (ITmr == TIM1)  { rccEnableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccEnableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccEnableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccEnableTIM4(FALSE); }
    else if(ITmr == TIM5)  { rccEnableTIM5(FALSE); }
    else if(ITmr == TIM6)  { rccEnableTIM6(FALSE); }
    else if(ITmr == TIM7)  { rccEnableTIM7(FALSE); }
    else if(ITmr == TIM8)  { rccEnableTIM8(FALSE); }
    else if(ITmr == TIM15) { rccEnableTIM15(FALSE); }
    else if(ITmr == TIM16) { rccEnableTIM16(FALSE); }
    else if(ITmr == TIM17) { rccEnableTIM17(FALSE); }
#endif
}

void Timer_t::Deinit() const {
    TMR_DISABLE(ITmr);
#if defined STM32F0XX
    if     (ITmr == TIM1)  { rccDisableTIM1(); }
#ifdef TIM2
    else if(ITmr == TIM2)  { rccDisableTIM2(); }
#endif
    else if(ITmr == TIM3)  { rccDisableTIM3(); }
#ifdef TIM6
    else if(ITmr == TIM6)  { rccDisableTIM6(); }
#endif
    else if(ITmr == TIM14) { rccDisableTIM14(); }
#ifdef TIM15
    else if(ITmr == TIM15) { rccDisableTIM15(); }
#endif
    else if(ITmr == TIM16) { rccDisableTIM16(); }
    else if(ITmr == TIM17) { rccDisableTIM17(); }
#elif defined STM32L4XX
    if     (ITmr == TIM1)  { rccDisableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccDisableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccDisableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccDisableTIM4(FALSE); }
    else if(ITmr == TIM5)  { rccDisableTIM5(FALSE); }
    else if(ITmr == TIM6)  { rccDisableTIM6(FALSE); }
    else if(ITmr == TIM7)  { rccDisableTIM7(FALSE); }
    else if(ITmr == TIM8)  { rccDisableTIM8(FALSE); }
    else if(ITmr == TIM15) { rccDisableTIM15(FALSE); }
    else if(ITmr == TIM16) { rccDisableTIM16(FALSE); }
    else if(ITmr == TIM17) { rccDisableTIM17(FALSE); }

#elif defined STM32F2XX || defined STM32F4XX
    if     (ITmr == TIM1)  { rccDisableTIM1(FALSE); }
    else if(ITmr == TIM2)  { rccDisableTIM2(FALSE); }
    else if(ITmr == TIM3)  { rccDisableTIM3(FALSE); }
    else if(ITmr == TIM4)  { rccDisableTIM4(FALSE); }
    else if(ITmr == TIM5)  { rccDisableTIM5(FALSE); }
    else if(ITmr == TIM6)  { rccDisableTIM6(FALSE); }
    else if(ITmr == TIM7)  { rccDisableTIM7(FALSE); }
    else if(ITmr == TIM8)  { rccDisableTIM8(FALSE); }
    else if(ITmr == TIM9)  { rccDisableTIM9(FALSE); }
    else if(ITmr == TIM11)  { rccDisableTIM11(FALSE); }
    else if(ITmr == TIM12)  { rccDisableTIM12(FALSE); }
    else if(ITmr == TIM14)  { rccDisableTIM14(FALSE); }
#endif
}

void PinOutputPWM_t::Init() const {
    Timer_t::Init();
    // GPIO
#if defined STM32L1XX
    AlterFunc_t AF = AF1; // For TIM2
    if(ANY_OF_2(ITmr, TIM3, TIM4)) AF = AF2;
    else if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) AF = AF3;
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF);
#elif defined STM32F0XX
    if     (ITmr == TIM1)  PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF2);
    else if(ITmr == TIM3)  PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
    else if(ITmr == TIM14) {
        if(ISetup.PGpio == GPIOA) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF4);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF0);
    }
#ifdef TIM15
    else if(ITmr == TIM15) {
        if(ISetup.PGpio == GPIOA) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF0);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
    }
#endif
    else if(ITmr == TIM16 or ITmr == TIM17) {
        if(ISetup.PGpio == GPIOA) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF5);
        else PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF2);
    }
#elif defined STM32F2XX || defined STM32F4XX
    if(ANY_OF_2(ITmr, TIM1, TIM2)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF1);
    else if(ANY_OF_3(ITmr, TIM3, TIM4, TIM5)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF2);
    else if(ANY_OF_4(ITmr, TIM8, TIM9, TIM10, TIM11)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF3);
    else if(ANY_OF_3(ITmr, TIM12, TIM13, TIM14)) PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF9);
#elif defined STM32F100_MCUCONF
    PinSetupAlterFunc(GPIO, N, OutputType, pudNone, AF0);   // Alternate function is dummy
#elif defined STM32L4XX
    AlterFunc_t AF = AF1;
    if(ITmr == TIM1 or ITmr == TIM2) AF = AF1;
    else if(ITmr == TIM3 or ITmr == TIM4 or ITmr == TIM5) AF = AF2;
    else if(ITmr == TIM8) AF = AF3;
    else if(ITmr == TIM15 or ITmr == TIM16 or ITmr == TIM17) AF = AF14;
    PinSetupAlterFunc(ISetup.PGpio, ISetup.Pin, ISetup.OutputType, pudNone, AF);
#endif
#if !defined STM32L151xB
    ITmr->BDTR = 0xC000;   // Main output Enable
#endif
    ITmr->ARR = ISetup.TopValue;
    // Setup Output
    uint16_t tmp = (ISetup.Inverted == invInverted)? 0b111 : 0b110; // PWM mode 1 or 2
    switch(ISetup.TimerChnl) {
        case 1:
            ITmr->CCMR1 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC1E;
            break;
        case 2:
            ITmr->CCMR1 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC2E;
            break;
        case 3:
            ITmr->CCMR2 |= (tmp << 4);
            ITmr->CCER  |= TIM_CCER_CC3E;
            break;
        case 4:
            ITmr->CCMR2 |= (tmp << 12);
            ITmr->CCER  |= TIM_CCER_CC4E;
            break;
        default: break;
    }
    Enable();
}

void Timer_t::SetUpdateFrequencyChangingPrescaler(uint32_t FreqHz) const {
    // Figure out input timer freq
    uint32_t InputFreq;
#if defined STM32L1XX
    // APB2
    if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;           // APB2CLK = HCLK / (not 1)
    }
    // APB1
    else {
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else  InputFreq = Clk.APB1FreqHz * 2;           // APB1CLK = HCLK / (not 1)
    }
#elif defined STM32L4XX
    // APB2
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM15, TIM16, TIM17)) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;            // APB2CLK = HCLK / (not 1)
    }
    // APB1
    else {
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else InputFreq = Clk.APB1FreqHz * 2;            // APB1CLK = HCLK / (not 1)
    }
#elif defined STM32F0XX
    if((RCC->CFGR & RCC_CFGR_PPRE_2) == 0) InputFreq = Clk.APBFreqHz; // APB1CLK = HCLK / 1
    else InputFreq = Clk.APBFreqHz * 2;                               // APB1CLK = HCLK / (not 1)
#elif defined STM32F2XX
    uint32_t Pre;
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM9, TIM10, TIM11)) {   // APB2
        Pre = (RCC->CFGR >> 13) & 0b111;
        InputFreq = Clk.APB2FreqHz;
    }
    else {
        Pre = (RCC->CFGR >> 10) & 0b111;
        InputFreq = Clk.APB1FreqHz;
    }
    if(Pre >= 0b100) InputFreq *= 2;
#endif
    uint32_t UpdFreqMax = InputFreq / (ITmr->ARR + 1);
    uint32_t div = UpdFreqMax / FreqHz;
    if(div != 0) div--;
//    Uart.Printf("InputFreq=%u; UpdFreqMax=%u; div=%u; ARR=%u\r", InputFreq, UpdFreqMax, div, ITmr->ARR);
    ITmr->PSC = div;
    ITmr->CNT = 0;  // Reset counter to start from scratch
}

void Timer_t::SetUpdateFrequencyChangingTopValue(uint32_t FreqHz) const {
    uint32_t InputFreq, TopVal;
#if defined STM32L1XX
    // APB2
    if(ANY_OF_3(ITmr, TIM9, TIM10, TIM11)) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;           // APB2CLK = HCLK / (not 1)
    }
    // APB1
    else {
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else  InputFreq = Clk.APB1FreqHz * 2;           // APB1CLK = HCLK / (not 1)
    }
#elif defined STM32L4XX
    // APB2
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM15, TIM16, TIM17)) {
        uint32_t APB2prs = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if(APB2prs < 0b100) InputFreq = Clk.APB2FreqHz; // APB2CLK = HCLK / 1
        else InputFreq = Clk.APB2FreqHz * 2;            // APB2CLK = HCLK / (not 1)
    }
    // APB1
    else {
        uint32_t APB1prs = (RCC->CFGR & RCC_CFGR_PPRE1) >> 8;
        if(APB1prs < 0b100) InputFreq = Clk.APB1FreqHz; // APB1CLK = HCLK / 1
        else InputFreq = Clk.APB1FreqHz * 2;            // APB1CLK = HCLK / (not 1)
    }
#elif defined STM32F2XX
    uint32_t Pre;
    if(ANY_OF_5(ITmr, TIM1, TIM8, TIM9, TIM10, TIM11)) {   // APB2
        Pre = (RCC->CFGR >> 13) & 0b111;
        InputFreq = Clk.APB2FreqHz;
    }
    else {
        Pre = (RCC->CFGR >> 10) & 0b111;
        InputFreq = Clk.APB1FreqHz;
    }
    if(Pre >= 0b100) InputFreq *= 2;
#elif defined STM32F0XX
//    InputFreq = *PClk;
#else
#error "Timer_t::SetUpdateFrequencyChangingTopValue: MCU not defined"
#endif
    TopVal  = (InputFreq / FreqHz) - 1;
//    Uart.Printf("Topval = %u\r", TopVal);
    SetTopValue(TopVal);
}
#endif


#if TIMER_KL // ======================= Virtual Timers =========================
// Universal VirtualTimer callback
void TmrKLCallback(void *p) {
    reinterpret_cast<TmrKL_t*>(p)->CallbackHandler();
}
#endif

// ================================= DEBUG =====================================
void chDbgPanic(const char *msg1) {
    Uart.PrintfNow("\r");
    Uart.PrintfNow(msg1);
    Uart.PrintfNow(" @");
    Uart.PrintfNow(chThdSelf()->p_name);
}

// ================================= Random ====================================
uint32_t Random(uint32_t TopValue) {
    rccEnableAHB2(RCC_AHB2ENR_RNGEN, FALSE);    // Enable clock
    RNG->CR |= RNG_CR_RNGEN;                    // Enable generator
    while(!(RNG->SR & RNG_SR_DRDY));            // Wait until ready
    uint32_t Rnd = RNG->DR;
    Rnd = Rnd % (TopValue + 1);
    rccDisableAHB2(RCC_AHB2ENR_RNGEN, FALSE);   // Stop clock
    return Rnd;
}

// =============================== I2C =========================================
void i2cDmaIrqHandler(void *p, uint32_t flags) {
    chSysLockFromISR();
    //Uart.Printf("===T===");
    Thread *PThd = ((i2c_t*)p)->PRequestingThread;
    if (PThd != NULL) {
        ((i2c_t*)p)->PRequestingThread = NULL;
        chSchReadyI(PThd);
    }
    chSysUnlockFromISR();
}

void i2c_t::Init(
        I2C_TypeDef *pi2c,
        GPIO_TypeDef *PGpio,
        uint16_t SclPin,
        uint16_t SdaPin,
        uint32_t BitrateHz,
        const stm32_dma_stream_t *APDmaTx,
        const stm32_dma_stream_t *APDmaRx
    ) {
    ii2c = pi2c;
    IPGpio = PGpio;
    ISclPin = SclPin;
    ISdaPin = SdaPin;
    IBitrateHz = BitrateHz;
    Standby();
    Resume();

    // ==== DMA ====
    // Here only unchanged parameters of the DMA are configured.
    DmaChnl = 3;   // I2C3
    if      (ii2c == I2C1) DmaChnl = 1;
    else if (ii2c == I2C2) DmaChnl = 7;

    // Setup Dma TX
    PDmaTx = APDmaTx;
    dmaStreamAllocate(PDmaTx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    dmaStreamSetPeripheral(PDmaTx, &ii2c->DR);
    // Setup Dma RX
    PDmaRx = APDmaRx;
    dmaStreamAllocate(PDmaRx, IRQ_PRIO_MEDIUM, i2cDmaIrqHandler, this);
    //dmaStreamAllocate(PDmaRx, 1, i2cDmaRXIrqHandler, NULL);
    dmaStreamSetPeripheral(PDmaRx, &ii2c->DR);
}

void i2c_t::Standby() {
    if      (ii2c == I2C1) { rccResetI2C1(); rccDisableI2C1(FALSE); }
    else if (ii2c == I2C2) { rccResetI2C2(); rccDisableI2C2(FALSE); }
    else if (ii2c == I2C3) { rccResetI2C3(); rccDisableI2C3(FALSE); }
    // Disable GPIOs
    PinSetupAnalog(IPGpio, ISclPin);
    PinSetupAnalog(IPGpio, ISdaPin);
}

void i2c_t::Resume() {
    Error = false;
    // ==== GPIOs ====
    PinSetupAlterFunc(IPGpio, ISclPin, omOpenDrain, pudNone, AF4);
    PinSetupAlterFunc(IPGpio, ISdaPin, omOpenDrain, pudNone, AF4);
    // ==== Clock and reset ====
    if      (ii2c == I2C1) { rccEnableI2C1(FALSE); rccResetI2C1(); }
    else if (ii2c == I2C2) { rccEnableI2C2(FALSE); rccResetI2C2(); }
    else if (ii2c == I2C3) { rccEnableI2C3(FALSE); rccResetI2C3(); }
    // Minimum clock is 2 MHz
    uint32_t ClkMhz = Clk.APB1FreqHz / 1000000;
    uint16_t tmpreg = ii2c->CR2;
    tmpreg &= (uint16_t)~I2C_CR2_FREQ;
    if(ClkMhz < 2)  ClkMhz = 2;
    if(ClkMhz > 30) ClkMhz = 30;
    tmpreg |= ClkMhz;
    ii2c->CR2 = tmpreg;
    ii2c->CR1 &= (uint16_t)~I2C_CR1_PE; // Disable i2c to setup TRise & CCR
    ii2c->TRISE = (uint16_t)(((ClkMhz * 300) / 1000) + 1);
    // 16/9
    tmpreg = (uint16_t)(Clk.APB1FreqHz / (IBitrateHz * 25));
    if(tmpreg == 0) tmpreg = 1; // minimum allowed value
    tmpreg |= I2C_CCR_FS | I2C_CCR_DUTY;
    ii2c->CCR = tmpreg;
    ii2c->CR1 |= I2C_CR1_PE;    // Enable i2c back
    // ==== DMA ====
    ii2c->CR2 |= I2C_CR2_DMAEN;
}

void i2c_t::Reset() {
    Standby();
    Resume();
}

uint8_t i2c_t::CmdWriteRead(uint8_t Addr,
        uint8_t *WPtr, uint8_t WLength,
        uint8_t *RPtr, uint8_t RLength) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength);
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    // Read if needed
    if(RLength != 0) {
        if(WaitEv8() != OK) return FAILURE;
        // Send repeated start
        SendStart();
        if(WaitEv5() != OK) return FAILURE;
        SendAddrWithRead(Addr);
        if(WaitEv6() != OK) { SendStop(); return FAILURE; }
        // If single byte is to be received, disable ACK before clearing ADDR flag
        if(RLength == 1) AckDisable();
        else AckEnable();
        ClearAddrFlag();
        dmaStreamSetMemory0(PDmaRx, RPtr);
        dmaStreamSetMode   (PDmaRx, I2C_DMARX_MODE);
        dmaStreamSetTransactionSize(PDmaRx, RLength);
        DmaLastTransferSet(); // Inform DMA that this is last transfer => do not ACK last byte
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaRx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaRx);
    } // if != 0
    SendStop();
    return OK;
}

uint8_t i2c_t::CmdWriteWrite(uint8_t Addr,
        uint8_t *WPtr1, uint8_t WLength1,
        uint8_t *WPtr2, uint8_t WLength2) {
    if(IBusyWait() != OK) return FAILURE;
    // Clear flags
    ii2c->SR1 = 0;
    while(RxIsNotEmpty()) (void)ii2c->DR;   // Read DR until it empty
    ClearAddrFlag();
    // Start transmission
    SendStart();
    if(WaitEv5() != OK) return FAILURE;
    SendAddrWithWrite(Addr);
    if(WaitEv6() != OK) { SendStop(); return FAILURE; }
    ClearAddrFlag();
    // Start TX DMA if needed
    if(WLength1 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr1);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength1);
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    if(WLength2 != 0) {
        if(WaitEv8() != OK) return FAILURE;
        dmaStreamSetMemory0(PDmaTx, WPtr2);
        dmaStreamSetMode   (PDmaTx, I2C_DMATX_MODE);
        dmaStreamSetTransactionSize(PDmaTx, WLength2);
        chSysLock();
        PRequestingThread = chThdSelf();
        dmaStreamEnable(PDmaTx);
        chSchGoSleepS(THD_STATE_SUSPENDED); // Sleep until end
        chSysUnlock();
        dmaStreamDisable(PDmaTx);
    }
    WaitBTF();
    SendStop();
    return OK;
}

// ==== Flag operations ====
// Busy flag
uint8_t i2c_t::IBusyWait() {
    uint8_t RetryCnt = 4;
    while(RetryCnt--) {
        if(!(ii2c->SR2 & I2C_SR2_BUSY)) return OK;
        chThdSleepMilliseconds(1);
    }
    Error = true;
    return TIMEOUT;
}

// BUSY, MSL & SB flags
uint8_t i2c_t::WaitEv5() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--) {
        uint16_t Flag1 = ii2c->SR1;
        uint16_t Flag2 = ii2c->SR2;
        if((Flag1 & I2C_SR1_SB) and (Flag2 & (I2C_SR2_MSL | I2C_SR2_BUSY))) return OK;
    }
    Error = true;
    return FAILURE;
}

uint8_t i2c_t::WaitEv6() {
    uint32_t RetryCnt = 45;
    uint16_t Flag1;
    do {
        Flag1 = ii2c->SR1;
        if((RetryCnt-- == 0) or (Flag1 & I2C_SR1_AF)) return FAILURE;   // Fail if timeout or NACK
    } while(!(Flag1 & I2C_SR1_ADDR)); // ADDR set when Address is sent and ACK received
    return OK;
}

uint8_t i2c_t::WaitEv8() {
    uint32_t RetryCnt = 45;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_TXE) return OK;
    Error = true;
    return TIMEOUT;
}

uint8_t i2c_t::WaitRx() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_RXNE) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitStop() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->CR1 & I2C_CR1_STOP) return OK;
    return TIMEOUT;
}

uint8_t i2c_t::WaitBTF() {
    uint32_t RetryCnt = 450;
    while(RetryCnt--)
        if(ii2c->SR1 & I2C_SR1_BTF) return OK;
    return TIMEOUT;
}
