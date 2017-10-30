/*
 * LEDs.cpp
 *
 *  Created on: 26 мая 2017 г.
 *      Author: Elessar
 */
#include "LEDs.h"

IntelLeds_t LedWs;

uint32_t ExcludingRandom(uint32_t ALowInclusive, uint32_t AHighInclusive, uint32_t AExcludingInclusive) {
    uint32_t Result = ALowInclusive;
    if (AHighInclusive-ALowInclusive == 1 and AExcludingInclusive == ALowInclusive)
        Result = AHighInclusive;
    else if (AHighInclusive-ALowInclusive > 1)
        do {
            Result = Random::Generate(ALowInclusive, AHighInclusive);
        } while (Result == AExcludingInclusive);
    return Result;
}

#if Attenuations_EN
#define Attenuations(brightness, percent)    (brightness-(uint16_t)(brightness*percent)/100)
//static inline uint8_t Attenuations(uint8_t ABrightness, uint8_t APercent) {
//    uint8_t Result = ABrightness - (uint16_t)(ABrightness*APercent)/100;
//    Uart.PrintfNow("AttenResult = %u (Brightness %u, Percent %u)\r", Result, ABrightness, APercent);
//    return Result;
//}
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

    Intensity_MIN.R = MINintensityR;
    Intensity_MIN.G = MINintensityG;
    Intensity_MIN.B = MINintensityB;
    Intensity_MAX.R = MAXintensityR;
    Intensity_MAX.G = MAXintensityG;
    Intensity_MAX.B = MAXintensityB;
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
            CurrNum.W = ExcludingRandom(0, LED_CNT-1, PrevNum.W);
            Val = Attenuations(Intensity_MIN.W, Attenuation.W);
            DesiredClr[PrevNum.W].R = Val;
            DesiredClr[PrevNum.W].G = Val;
            DesiredClr[PrevNum.W].B = Val;

            Val = Attenuations(Random::Generate(Intensity_MIN.W, Intensity_MIN.W), Attenuation.W);
            DesiredClr[CurrNum.W].R = Val;
            DesiredClr[CurrNum.W].G = Val;
            DesiredClr[CurrNum.W].B = Val;

            Val = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.W].R, Val, Random::Generate(MINproc_time, MAXproc_time));
            StepIntensity[CurrNum.W].R = Val;
            StepIntensity[CurrNum.W].G = Val;
            StepIntensity[CurrNum.W].B = Val;

            Val = Random::Generate(MINpause, MAXpause);
            PauseCounter[CurrNum.W].R = Val;
            PauseCounter[CurrNum.W].G = Val;
            PauseCounter[CurrNum.W].B = Val;
            break;
        case prColor:
            CurrNum.R = ExcludingRandom(0, LED_CNT-1, PrevNum.R);
            CurrNum.G = ExcludingRandom(0, LED_CNT-1, PrevNum.G);
            CurrNum.B = ExcludingRandom(0, LED_CNT-1, PrevNum.B);

            DesiredClr[PrevNum.R].R = Attenuations(Intensity_MIN.R, Attenuation.R);
            DesiredClr[PrevNum.G].G = Attenuations(Intensity_MIN.G, Attenuation.G);
            DesiredClr[PrevNum.B].B = Attenuations(Intensity_MIN.B, Attenuation.B);

            DesiredClr[CurrNum.R].R = Attenuations(Random::Generate(Intensity_MIN.R, Intensity_MAX.R), Attenuation.R);
            DesiredClr[CurrNum.G].G = Attenuations(Random::Generate(Intensity_MIN.G, Intensity_MAX.G), Attenuation.G);
            DesiredClr[CurrNum.B].B = Attenuations(Random::Generate(Intensity_MIN.B, Intensity_MAX.B), Attenuation.B);

            StepIntensity[CurrNum.R].R = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.R].R, DesiredClr[CurrNum.R].R, Random::Generate(MINproc_time, MAXproc_time));
            StepIntensity[CurrNum.G].G = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.G].G, DesiredClr[CurrNum.G].G, Random::Generate(MINproc_time, MAXproc_time));
            StepIntensity[CurrNum.B].B = CalculateStepIntensity(LedWs.ICurrentClr[CurrNum.B].B, DesiredClr[CurrNum.B].B, Random::Generate(MINproc_time, MAXproc_time));
            Uart.PrintfNow("DesiredClr R%u G%u B%u\r", DesiredClr[CurrNum.R].R, DesiredClr[CurrNum.G].G, DesiredClr[CurrNum.B].B);
//            Uart.PrintfNow("StepIntensity R%u G%u B%u\r", StepIntensity[CurrNum.R].R, StepIntensity[CurrNum.G].G, StepIntensity[CurrNum.B].B);
//            Uart.PrintfNow("  IntensProcT R%u G%u B%u\r", StepIntensity[CurrNum.R].R*DesiredClr[CurrNum.R].R, StepIntensity[CurrNum.G].G*DesiredClr[CurrNum.G].G, StepIntensity[CurrNum.B].B*DesiredClr[CurrNum.B].B);

            PauseCounter[CurrNum.R].R = Random::Generate(MINpause, MAXpause);
            PauseCounter[CurrNum.G].G = Random::Generate(MINpause, MAXpause);
            PauseCounter[CurrNum.B].B = Random::Generate(MINpause, MAXpause);
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
