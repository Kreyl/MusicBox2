/*
 * SnsPins.h
 *
 *  Created on: 17 џэт. 2015 у.
 *      Author: Kreyl
 */

/* ================ Documentation =================
 * There are several (may be 1) groups of sensors (say, buttons and USB connection).
 *
 */

#pragma once

#include "SimpleSensors.h"

#ifndef SIMPLESENSORS_ENABLED
#define SIMPLESENSORS_ENABLED   FALSE
#endif

#if SIMPLESENSORS_ENABLED
#define SNS_POLL_PERIOD_MS      72

// Button handler
extern void ProcessButtons(PinSnsState_t *PState, uint32_t Len);
extern void Process5VSns(PinSnsState_t *PState, uint32_t Len);
extern void Process3VSns1(PinSnsState_t *PState, uint32_t Len);
extern void Process3VSns2(PinSnsState_t *PState, uint32_t Len);

const PinSns_t PinSns[] = {
        // Button
        {BTN_VolUp_pin, ProcessButtons},
        {BTN_VolDown_pin, ProcessButtons},
        // 5V sns
        {ExternalPWR_Pin, Process5VSns},
        {Sensor1_Pin, Process3VSns1},
        {Sensor2_Pin, Process3VSns2},
};
#define PIN_SNS_CNT     countof(PinSns)

#endif  // if enabled
