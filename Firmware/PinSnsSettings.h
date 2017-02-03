/*
 * SnsPins.h
 *
 *  Created on: 17 ���. 2015 �.
 *      Author: Kreyl
 */

/* ================ Documentation =================
 * There are several (may be 1) groups of sensors (say, buttons and USB connection).
 * There is GPIO and Pin data for every sensor.
 *
 */

#pragma once

#include "ch.h"
#include "hal.h"
#include "kl_lib_f2xx.h"
#include "board.h"

#include "main.h" // App.thd here
#include "evt_mask.h"

#ifndef SIMPLESENSORS_ENABLED
#define SIMPLESENSORS_ENABLED   FALSE
#endif
#define SNS_POLL_PERIOD_MS      100

#if SIMPLESENSORS_ENABLED
enum PinSnsState_t {pssNone, pssLo, pssHi, pssRising, pssFalling};
typedef void (*ftVoidPSnsStLen)(PinSnsState_t *PState, uint32_t Len);

// Single pin setup data
class PinSns_t : public PinInput_t {
public:
    ftVoidPSnsStLen Postprocessor;
    PinSns_t(PinInputSetup_t APin, ftVoidPSnsStLen pp) : PinInput_t(APin), Postprocessor(pp) {}
};

// ================================= Settings ==================================
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
