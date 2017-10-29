/*
 * LEDs.h
 *
 *  Created on: 26 мая 2017 г.
 *      Author: Elessar
 */
#pragma once

#include "ws2812b.h"
#include "evt_mask.h"
#include "board.h"
#include "LEDsConf.h"


#define Channels_CNT    LED_CNT * 3 // for RGB LEDs
#define LEDparamNone    UINT16_MAX
//#define Attenuations_EN true

enum LEDsProfile_t { prWhite, prColor, prEnd };

typedef struct {
    uint16_t R, G, B;
} Color16_t;
typedef struct {
    uint8_t R, G, B;
} Color8_t;


class LEDs_t : private IrqHandler_t {
private:
    thread_t *PAppThd;
    eventmask_t EvtEnd;
    virtual_timer_t ITmr;

    Color16_t PauseCounter[LED_CNT] = {{1}};
    Color16_t IntensityCounter[LED_CNT] = {{0}};
    Color16_t StepIntensity[LED_CNT] = {{1}};
    uint16_t *PPauseCounter, *PIntensityCounter, *PStepIntensity;
    uint8_t  Attenuation = 0;

    Color_t DesiredClr[LED_CNT];
    uint8_t *PDesiredClr, *PCurrentClr;
    Color_t CurrNum, PrevNum;  // Current Number & Previous Number
    uint8_t Profile;

    void IEndSequence() {
        if(PAppThd != nullptr) chEvtSignalI(PAppThd, EvtEnd);
    }
    uint16_t CalculateStepIntensity(uint8_t CurrentVal, uint8_t DesiredVal, uint16_t AProcessTime) {
        uint8_t Delta = ABS(DesiredVal - CurrentVal);
//        Uart.PrintfNow("Delta = %u (DesiredVal %u, CurrentVal %u)\r", Delta, DesiredVal, CurrentVal);
        if (AProcessTime > Delta and Delta != 0)
            return AProcessTime/Delta;
        else return 1;
    }
    void IProcessSequenceI();
    void IIrqHandler() { IProcessSequenceI(); }

public:
    void Init();
    void SetupSeqEndEvt(eventmask_t AEvt) {
        PAppThd = chThdGetSelfX();
        EvtEnd = AEvt;
    }
    void Start() { chVTSet(&ITmr, MS2ST(1), TmrKLCallback, this); }
    void Stop() { chVTReset(&ITmr); }

    void SetProfile(LEDsProfile_t AProfile) { Profile = AProfile; }
    uint8_t GetProfile() { return Profile; }
    void ProfToggle() {
        if (Profile < prEnd-1)
            Profile ++;
        else
            Profile = 0;
    }

    void SetAll(uint16_t AIntensity, uint16_t ASpeed, uint16_t APause);
    void SetByNumber(uint16_t AIntensity, uint16_t ASpeed, uint16_t APause, uint8_t ANumber);
    void GenerationParam();
    bool IsProcessing() {
        chSysLock();
        bool Result = false;
        for (uint8_t i=0; i<Channels_CNT; i++) {
            if (PPauseCounter[i] != 0) {
                Result = true;
                break;
            }
        }
        chSysUnlock();
        return Result;
    };
    void SetAttenuation(uint8_t APercent) { Attenuation = APercent; }
};
