/*
 * LEDs.cpp
 *
 *  Created on: 26 мая 2017 г.
 *      Author: Elessar
 */
#include "LEDs.h"

IntelLeds_t LedWs;

static inline int ExcludingRandom(int LowInclusive, int HighInclusive, int ExcludingInclusive) {
    int Result;
    do {
        Result = (rand() % (HighInclusive + 1 - LowInclusive)) + LowInclusive;
    } while (Result == ExcludingInclusive);
    return Result;
}
#if Attenuations_EN
//#define Attenuations(brightness, percent)    (brightness-(uint16_t)(brightness*percent+1)/100)
static inline uint8_t Attenuations(uint8_t ABrightness, uint8_t APercent) {
    uint8_t Result = ABrightness - (uint16_t)(ABrightness*APercent)/100;
//    Uart.PrintfNow("AttenResult = %u (Brightness %u, Percent %u)\r", Result, ABrightness, APercent);
    return Result;
}
#else
#define Attenuations(brightness, percent)    (brightness)       // пустышка
#endif


void LEDs_t::Init() {
    LedWs.Init();
//    LedWs.ISetCurrentColors();
    PDesiredClr = &DesiredClr[0].R;
    PCurrentClr = &LedWs.ICurrentClr[0].R;
    PPauseCounter = &PauseCounter[0].R;
    PIntensityCounter = &IntensityCounter[0].R;
    PStepIntensity = &StepIntensity[0].R;
}

// ==== Process sequence ====
void LEDs_t::IProcessSequenceI() {
    chVTSetI(&ITmr, MS2ST(1), TmrKLCallback, this);
    bool UpdData = false;
    for (uint8_t i=0; i<Channels_CNT; i++) {
        if (PCurrentClr[i] != PDesiredClr[i]) {
            if (PIntensityCounter[i] == 0) {
                PIntensityCounter[i] = PStepIntensity[i];
                if (PCurrentClr[i] < PDesiredClr[i])
                    PCurrentClr[i] ++;
                else
                    PCurrentClr[i] --;
                UpdData = true;
            } else
                PIntensityCounter[i] --;
        } else if (PPauseCounter[i] != 0) {
            PPauseCounter[i] --;
            if ( (PauseCounter[CurrNum.R].R+PauseCounter[CurrNum.G].G+PauseCounter[CurrNum.B].B)==0 )
                IEndSequence();
        }
    }
    if (UpdData) LedWs.ISetCurrentColors();
}

void LEDs_t::GenerationParam() {
    chSysLock();
    PrevNum = CurrNum;
    uint16_t Val;
    switch(Profile) {
        case prWhite:
            CurrNum.R = ExcludingRandom(0, LED_CNT-1, PrevNum.R);
            DesiredClr[PrevNum.R].R = Attenuations(MINintensity, Attenuation);
            DesiredClr[PrevNum.R].G = Attenuations(MINintensity, Attenuation);
            DesiredClr[PrevNum.R].B = Attenuations(MINintensity, Attenuation);

            Val = Attenuations(Random(MINintensity, MAXintensity), Attenuation);
            DesiredClr[CurrNum.R].R = Val;
            DesiredClr[CurrNum.R].G = Val;
            DesiredClr[CurrNum.R].B = Val;

            Val = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.R].R, Val, Random(MINproc_time, MAXproc_time));
            StepIntensity[CurrNum.R].R = Val;
            StepIntensity[CurrNum.R].G = Val;
            StepIntensity[CurrNum.R].B = Val;

            Val = Random(MINpause, MAXpause);
            PauseCounter[CurrNum.R].R = Val;
            PauseCounter[CurrNum.R].G = Val;
            PauseCounter[CurrNum.R].B = Val;
            break;
        case prColor:
            CurrNum.R = ExcludingRandom(0, LED_CNT-1, PrevNum.R);
            CurrNum.G = ExcludingRandom(0, LED_CNT-1, PrevNum.G);
            CurrNum.B = ExcludingRandom(0, LED_CNT-1, PrevNum.B);

            DesiredClr[PrevNum.R].R = Attenuations(MINintensityR, Attenuation);
            DesiredClr[PrevNum.G].G = Attenuations(MINintensityG, Attenuation);
            DesiredClr[PrevNum.B].B = Attenuations(MINintensityB, Attenuation);

            DesiredClr[CurrNum.R].R = Attenuations(Random(MINintensityR, MAXintensityR), Attenuation);
            DesiredClr[CurrNum.G].G = Attenuations(Random(MINintensityG, MAXintensityG), Attenuation);
            DesiredClr[CurrNum.B].B = Attenuations(Random(MINintensityB, MAXintensityB), Attenuation);

            StepIntensity[CurrNum.R].R = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.R].R, DesiredClr[CurrNum.R].R, Random(MINproc_time, MAXproc_time));
            StepIntensity[CurrNum.G].G = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.G].G, DesiredClr[CurrNum.G].G, Random(MINproc_time, MAXproc_time));
            StepIntensity[CurrNum.B].B = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.B].B, DesiredClr[CurrNum.B].B, Random(MINproc_time, MAXproc_time));
//            Uart.PrintfNow("DesiredClr R%u G%u B%u\r", DesiredClr[CurrNum.R].R, DesiredClr[CurrNum.G].G, DesiredClr[CurrNum.B].B);
//            Uart.PrintfNow("StepIntensity R%u G%u B%u\r", StepIntensity[CurrNum.R].R, StepIntensity[CurrNum.G].G, StepIntensity[CurrNum.B].B);
//            Uart.PrintfNow("  IntensProcT R%u G%u B%u\r", StepIntensity[CurrNum.R].R*DesiredClr[CurrNum.R].R, StepIntensity[CurrNum.G].G*DesiredClr[CurrNum.G].G, StepIntensity[CurrNum.B].B*DesiredClr[CurrNum.B].B);

            PauseCounter[CurrNum.R].R = Random(MINpause, MAXpause);
            PauseCounter[CurrNum.G].G = Random(MINpause, MAXpause);
            PauseCounter[CurrNum.B].B = Random(MINpause, MAXpause);
            break;
        default: break;
    }
    chSysUnlock();
//    Uart.Printf("PrevNum.R: %u, CurrNum.R: %u\r", PrevNum.R, CurrNum.R);
}

void LEDs_t::SetAll(uint16_t AIntensity, uint16_t AProcessTime, uint16_t APause) {
    chSysLock();
    for (uint8_t i=0; i<Channels_CNT; i++) {
        if (AIntensity != LEDparamNone)
            PDesiredClr[i] = AIntensity;
        if (AProcessTime != LEDparamNone)
            PStepIntensity[i] = CalculateStepIntensity(PCurrentClr[i], AIntensity, AProcessTime);
//        Uart.PrintfNow("StepIntensity[%u]: %u\r", i, PStepIntensity[i]);
        if (APause != LEDparamNone)
            PPauseCounter[i] = APause + 1;
    }
    chSysUnlock();
}

void LEDs_t::SetByNumber(uint16_t AIntensity, uint16_t AProcessTime, uint16_t APause, uint8_t ANumber) {
    chSysLock();
    PDesiredClr[ANumber] = AIntensity;
    PStepIntensity[ANumber] = CalculateStepIntensity(PCurrentClr[ANumber], AIntensity, AProcessTime);
    PPauseCounter[ANumber] = APause + 1;
    chSysUnlock();
}
