/*
 * RotaryDial.cpp
 *
 *  Created on: 07 ����. 2017 �.
 *      Author: Elessar
 */

#include "RotaryDial.h"

Dial_t Dialer;
PinIrq_t DialIRQ(Dial_Namber_GPIO, Dial_Namber_PIN, pudPullUp);

void LockTmrCallback(void *p) {
    chSysLockFromISR();
    ((Dial_t*)p)->IProcessSequenceI(deUnlockIRQ);
    chSysUnlockFromISR();
}
void DiskTmrCallback(void *p) {
    chSysLockFromISR();
    ((Dial_t*)p)->IProcessSequenceI(deDiskPoll);
    chSysUnlockFromISR();
}
void SendEvtTmrCallback(void *p) {
    chSysLockFromISR();
    ((Dial_t*)p)->IProcessSequenceI(deSendEvt);
    chSysUnlockFromISR();
}

// ================================= IRQ =======================================
extern "C" {
// IRQ
CH_IRQ_HANDLER(DIAL_IRQ_HANDLER) {
//    Uart.PrintfNow("IRQ %d %d\r", ch.dbg.isr_cnt, ch.dbg.lock_cnt);
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    Dialer.IIrqPinHandler();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}
} // extern c
void Dial_t::IIrqPinHandler() {     // Interrupt caused by Low level on IRQ_Pin
    DialIRQ.CleanIrqFlag();         // Clear IRQ Pending Bit
    DialIRQ.DisableIrq();           // Disable IRQ
    Numeral ++;
//    Uart.PrintfI("  IRQ nam %u\r", Numeral);
    chVTSetI(&LockTmr, MS2ST(IRQ_En_Delay_MS), LockTmrCallback, this);
}

// =========================== Implementation ==================================

void Dial_t::IProcessSequenceI(DialerEvt_t DialerEvt) {
    switch(DialerEvt) {
        case deUnlockIRQ:
            DialIRQ.EnableIrq(IRQ_PRIO_LOW);
            break;
        case deDiskPoll:
            if (DiskIsArmed() and !DiskWasArmed) {
                DiskWasArmed = true;
//                Uart.PrintfI("  Disk armed\r");
                if (chVTIsArmedI(&SendEvtTmr)) chVTResetI(&SendEvtTmr);
            } else if (!DiskIsArmed() and DiskWasArmed) {
                DiskWasArmed = false;
//                Uart.PrintfI("  Disk was armed\r");
                CalculateNumber();
                chVTSetI(&SendEvtTmr, MS2ST(EndNumber_TimeOut_MS), SendEvtTmrCallback, this);
            }
            chVTSetI(&DiskTmr, MS2ST(Disk_Poll_Period_MS), DiskTmrCallback, this);
            break;
        case deSendEvt:
            if(IPAppThd != nullptr) chEvtSignalI(IPAppThd, EvtEnd);
            break;
        default: break;
    }
}

void Dial_t::Init() {
    DialIRQ.Init(ttRising);
    PinSetupInput(Dial_Disk_GPIO, Dial_Disk_PIN, pudPullUp);
    chSysLock();
    chVTSetI(&DiskTmr, MS2ST(Disk_Poll_Period_MS), DiskTmrCallback, this);
    DialIRQ.CleanIrqFlag();
    DialIRQ.EnableIrq(IRQ_PRIO_LOW); // Enable dreq irq
    chSysUnlock();
}

