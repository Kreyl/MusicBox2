/*
 * main.h
 *
 *  Created on: 30.03.2013
 *      Author: Elessar
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"
#include "hal.h"
#include "kl_lib.h"
#include "uart.h"
#include "evt_mask.h"
#include "kl_sd.h"
#include "ff.h"
#include "MassStorage.h"
#include "battery_consts.h"
#include "board.h"


//enum AppState_t  {
//    asDefault,
//};

class App_t {
private:
    thread_t *PThread; // Main thread
//    void EnterState(AppState_t NewState);
//    AppState_t SystemState = asDefault;

public:
    void InitThread() { PThread = chThdGetSelfX(); }
    void SignalEvt(uint32_t EvtMsk) {
        chSysLock();
        chEvtSignalI(PThread, EvtMsk);
        chSysUnlock();
    }
    void SignalEvtI(eventmask_t Evt) { chEvtSignalI(PThread, Evt); }
    void OnCmd(Shell_t *PShell);
    // Inner use

    void PowerON();
    void ShutDown();
    void ITask();
};
extern App_t App;



class Periphy_t {
private:

public:
    void InitSwich(){
        PinSetupOut(PeriphySW_Pin, PeriphySW_PinMode);
        PinSetHi(PeriphySW_Pin);
        PinSetupOut(PeriphyPWSW_Pin, PeriphySW_PinMode);
        PinSetHi(PeriphyPWSW_Pin);
    }
    void ON() { PinSetLo(PeriphySW_Pin); PinSetLo(PeriphyPWSW_Pin); }
    void OFF() { PinSetHi(PeriphySW_Pin); PinSetHi(PeriphyPWSW_Pin); }
//    bool _5V_is_here() { return PinIsSet(GPIOA, 9); }//GPIOA, 9, pudPullDown    ExternalPWR_Pin
};
extern Periphy_t Periphy;


#endif /* MAIN_H_ */
