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
#define Attenuations_EN true

#if Attenuations_EN
#define Attenuations(brightness, percent)    (brightness-(uint16_t)(brightness*percent)/100)
#else
#define Attenuations(brightness, percent)    (brightness)       // пустышка
#endif

#define MAX_VAL         255

enum LEDsProfile_t { prWhite, prColor, prEnd };

typedef union {
    uint16_t W;
    struct {
        uint16_t R, G, B;
    };
} Color16_t;
typedef union {
    uint8_t W;
    struct {
        uint8_t R, G, B;
    };
} Color8_t;
typedef enum {cpR, cpG, cpB, cpW, cpRGB} ColorParams_t;

class LEDs_t : private IrqHandler_t {
private:
    thread_t *PAppThd;
    eventmask_t EvtEnd;
    virtual_timer_t ITmr;

    Color16_t PauseCounter[LED_CNT] = {{0}};
    Color16_t IntensityCounter[LED_CNT] = {{0}};
    Color16_t StepIntensity[LED_CNT] = {{1}};
    uint16_t *PPauseCounter, *PIntensityCounter, *PStepIntensity;
    Color8_t Attenuation = {0};
    Color8_t Intensity_MIN, Intensity_MAX;

    Color8_t DesiredClr[LED_CNT];
    uint8_t *PDesiredClr, *PCurrentClr;
    Color8_t CurrNum, PrevNum;  // Current Number & Previous Number
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
    void SetByChannelNumber(uint16_t AIntensity, uint16_t ASpeed, uint16_t APause, uint8_t ANumber);
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

    void SetAttenuation(uint8_t APercent, ColorParams_t Params = cpRGB) {
        if (APercent > 100) APercent = 100;
        switch(Params) {
            case cpR: Attenuation.R = APercent; break;
            case cpG: Attenuation.G = APercent; break;
            case cpB: Attenuation.B = APercent; break;
            case cpW: Attenuation.W = APercent; break;
            case cpRGB:
                Attenuation.R = APercent;
                Attenuation.G = APercent;
                Attenuation.B = APercent;
                break;
        }
    }
    void SetIntensityLevel(uint8_t APercent, ColorParams_t Params = cpRGB) {
        if (APercent < 100)
            APercent = 100 - APercent;
        else
            APercent = 0;
        switch(Params) {
            case cpR: Attenuation.R = APercent; break;
            case cpG: Attenuation.G = APercent; break;
            case cpB: Attenuation.B = APercent; break;
            case cpW: Attenuation.W = APercent; break;
            case cpRGB:
                Attenuation.R = APercent;
                Attenuation.G = APercent;
                Attenuation.B = APercent;
                break;
        }
    }
    void SetLimit_MIN(uint8_t APercent, ColorParams_t Params = cpRGB) {
        if (APercent < 100)
            APercent = 100 - APercent;
        else
            APercent = 0;
        switch(Params) {
            case cpR: Intensity_MIN.R = Attenuations(MAX_VAL, APercent); break;
            case cpG: Intensity_MIN.G = Attenuations(MAX_VAL, APercent); break;
            case cpB: Intensity_MIN.B = Attenuations(MAX_VAL, APercent); break;
            case cpW: Intensity_MIN.W = Attenuations(MAX_VAL, APercent); break;
            case cpRGB:
                Intensity_MIN.R = Attenuations(MAX_VAL, APercent);
                Intensity_MIN.G = Attenuations(MAX_VAL, APercent);
                Intensity_MIN.B = Attenuations(MAX_VAL, APercent);
                Uart.Printf("  Limit_MIN=%d\r ", Attenuations(MAX_VAL, APercent));
                break;
        }
    }
    void SetLimit_MAX(uint8_t APercent, ColorParams_t Params = cpRGB) {
        if (APercent < 100)
            APercent = 100 - APercent;
        else
            APercent = 0;
        switch(Params) {
            case cpR: Intensity_MAX.R = Attenuations(MAX_VAL, APercent); break;
            case cpG: Intensity_MAX.G = Attenuations(MAX_VAL, APercent); break;
            case cpB: Intensity_MAX.B = Attenuations(MAX_VAL, APercent); break;
            case cpW: Intensity_MAX.W = Attenuations(MAX_VAL, APercent); break;
            case cpRGB:
                Intensity_MAX.R = Attenuations(MAX_VAL, APercent);
                Intensity_MAX.G = Attenuations(MAX_VAL, APercent);
                Intensity_MAX.B = Attenuations(MAX_VAL, APercent);
                Uart.Printf("  Limit_MAX=%d\r ", Attenuations(MAX_VAL, APercent));
                break;
        }
    }

};
