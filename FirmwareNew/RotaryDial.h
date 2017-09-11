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
#else
#error "RotaryDial.h: not selected IRQ pin"
#endif


#define IRQ_En_Delay_MS         20
#define Disk_Poll_Period_MS     100
#define EndNumber_TimeOut_MS    1500

typedef enum {deUnlockIRQ, deDiskPoll, deSendEvt} DialerEvt_t;

// VirtualTimers callback
void LockTmrCallback(void *p);
void DiskTmrCallback(void *p);
void SendEvtTmrCallback(void *p);

class Dial_t {
private:
    uint8_t Numeral = 0;
    uint64_t Number = 0;
    virtual_timer_t LockTmr, DiskTmr, SendEvtTmr;
    thread_t *IPAppThd;
    eventmask_t EvtEnd;
    bool DiskWasArmed = false;
    bool DiskIsArmed() {
        return PinIsLo(Dial_Disk_GPIO, Dial_Disk_PIN);
    }
    void CalculateNumber() {
        if (Numeral == 10) Numeral = 0;
        Number = Number*10 + Numeral;
        Numeral = 0;
    }
public:
    void Init();
    void SetupSeqEndEvt(eventmask_t AEvt) {
        IPAppThd = chThdGetSelfX();
        EvtEnd = AEvt;
    }
    uint64_t GetNumber() {
        uint64_t temp = Number;
//        Uart.Printf("Namber %u\r", Number);
        Number = 0;
        return temp;
    }
    // Inner use
    void IProcessSequenceI(DialerEvt_t DialerEvt);
    inline void IIrqPinHandler();
};

extern Dial_t Dialer;
