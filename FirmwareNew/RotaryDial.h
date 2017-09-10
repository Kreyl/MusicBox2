/*
 * RotaryDial.h
 *
 *  Created on: 07 сент. 2017 г.
 *      Author: Elessar
 */

#pragma once

#include "uart.h"
#include "board.h"
#include "kl_lib.h"

// === EXTI IRQ Handler ===
#if Dial_Namber_PIN == 0
#define DIAL_IRQ_HANDLER  Vector58
#elif Dial_Namber_PIN == 1
#define DIAL_IRQ_HANDLER  Vector5C
#elif Dial_Namber_PIN == 2
#define DIAL_IRQ_HANDLER  Vector60
#elif Dial_Namber_PIN == 3
#define DIAL_IRQ_HANDLER  Vector64
#elif Dial_Namber_PIN == 4
#define DIAL_IRQ_HANDLER  Vector68
#endif

#define IRQ_En_Delay_MS         20
#define Disk_Poll_Period_MS     100
#define EndNamber_TimeOut_MS    1000

typedef enum {deUnlockIRQ, deDiskPoll, deSendEvt} DialerEvt_t;

// VirtualTimers callback
void LockTmrCallback(void *p);
void DiskTmrCallback(void *p);
void SendEvtTmrCallback(void *p);

class Dialer_t {
private:
    uint8_t Namber = 0;
    virtual_timer_t LockTmr, DiskTmr, SendEvtTmr;
    thread_t *IPAppThd;
    eventmask_t EvtEnd;
    bool DiskWasArmed = false;
    bool DiskIsArmed() {
        return PinIsLo(Dial_Disk_GPIO, Dial_Disk_PIN);
    }
public:
    void Init();
    void SetupSeqEndEvt(eventmask_t AEvt) {
        IPAppThd = chThdGetSelfX();
        EvtEnd = AEvt;
    }
    uint32_t GetNamber() {
        return Namber;
    }
    // Inner use
    void IProcessSequenceI(DialerEvt_t DialerEvt);
    inline void IIrqPinHandler();
};

extern Dialer_t Dialer;
