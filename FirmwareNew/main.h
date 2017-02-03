/*
 * main.h
 *
 *  Created on: 30.03.2013
 *      Author: Elessar
 */

#ifndef MUSICBOX2_NEW_MAIN_H_
#define MUSICBOX2_NEW_MAIN_H_

#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "uart.h"
#include "evt_mask.h"
#include "kl_sd.h"
#include "ff.h"
#include "MassStorage.h"
#include "adc_f2.h"
#include "battery_consts.h"
#include "board.h"


// External Power Input
#define PWR_EXTERNAL_GPIO   GPIOA
#define PWR_EXTERNAL_PIN    9


enum AppState_t  {
    asDefault,
};

class App_t {
private:
    bool WasExternal = false;
    thread_t *PThread; // Main thread
    void EnterState(AppState_t NewState);
    AppState_t SystemState = asDefault;
    //  === ===
//    VirtualTimer IOpenK1KorTmr;

public:
    void InitThread() { PThread = chThdGetSelfX(); } //chThdSelf()
    void SignalEvt(uint32_t EvtMsk) {
        chSysLock();
        chEvtSignalI(PThread, EvtMsk);
        chSysUnlock();
    }
    void SignalEvtI(eventmask_t Evt) { chEvtSignalI(PThread, Evt); }
    void OnCmd(Shell_t *PShell);
    // Inner use
    inline bool ExternalPwrOn() { return  PinIsHi(PWR_EXTERNAL_GPIO, PWR_EXTERNAL_PIN); }
    void ITask();
};
extern App_t App;


#endif /* MUSICBOX2_NEW_MAIN_H_ */
