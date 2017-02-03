/*
 * File:   main.cpp
 * Author: Elessar
 * Project: MasonOrder
 *
 * Created on May 27, 2016, 6:37 PM
 */


#include "main.h"
#include "sound.h"
#include "Soundlist.h"
#include "Sequences.h"
//#include "buttons.h"
//#include "SimpleSensors.h"
//#include "WS2812BforSPI.h"
//#include "WS2812BforTMR.h"

App_t App;
SndList_t SndList;

// =============================== Main ========================================
int main() {
    // ==== Init ====
    // ==== Setup clock ====
    Clk.UpdateFreqValues();
    uint8_t ClkResult = FAILURE;
    Clk.SetupFlashLatency(12);  // Setup Flash Latency for clock in MHz
    // 12 MHz/6 = 2; 2*192 = 384; 384/8 = 48 (preAHB divider); 384/8 = 48 (USB clock)
    Clk.SetupPLLDividers(6, 192, pllSysDiv8, 8);
    // 48/4 = 12 MHz core clock. APB1 & APB2 clock derive on AHB clock
    Clk.SetupBusDividers(ahbDiv4, apbDiv1, apbDiv1);
    if((ClkResult = Clk.SwitchToPLL()) == 0) Clk.HSIDisable();
    Clk.UpdateFreqValues();

    // ==== Init OS ====
    halInit();
    chSysInit();

    // ==== Init Hard & Soft ====
    App.InitThread();
    Uart.Init(115200, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN);
    Uart.Printf("\r%S AHB freq=%uMHz\r", BOARD_NAME, Clk.AHBFreqHz/1000000);
    // Report problem with clock if any
    if(ClkResult) Uart.Printf("Clock failure\r");

    // USB related
    PinSetupInput(PWR_EXTERNAL_GPIO, PWR_EXTERNAL_PIN, pudPullDown);
    MassStorage.Init();

    SD.Init();      // пирание на SD подано посто€нно

    Sound.Init();
    Sound.AmpfOn();
    Sound.RegisterAppThd(chThdSelf());

    // Setup inputs
//    PinSetupIn(Box_GPIO, Box1_PIN, pudPullUp);
    // Setup outputs
//    PinSetupOut(StayAwake_GPIO, StayAwake_PIN, omPushPull, pudNone);

    // ===== Main cycle =====
    App.ITask();
}



__attribute__ ((__noreturn__))
void App_t::ITask() {
#if 0 // —оздать файл (проба)
    FIL File;
    FRESULT Rslt = f_open(&File, "ID_Store.ini",FA_READ+FA_OPEN_EXISTING+FA_CREATE_NEW);//
    if(Rslt != FR_OK) {
        if (Rslt == FR_NO_FILE) Uart.Printf("\r%S: not found", "ID_Store.ini");
        else Uart.Printf("\r%S: openFile error: %u", "ID_Store.ini", Rslt);
     }
    chThdSleepMilliseconds(999);
#endif
    if(!ExternalPwrOn()) {
        SndList.PlayRandomFileFromDir("0:\\");
    }
while(true) {
    // ∆дЄм событи€ EVTMSK_PLAY_ENDS (окончание воспроизведени€); если недождались в течении 306м— - идЄм дальше
//    eventmask_t EvtMsk = chEvtWaitAnyTimeout(EVTMSK_PLAY_ENDS, MS2ST(306));
    uint32_t EvtMsk = chEvtWaitAny(ALL_EVENTS);

    if((EvtMsk & EVTMSK_PLAY_ENDS) and !ExternalPwrOn()) {
        SndList.PlayRandomFileFromDir("0:\\");
    }


    if(EvtMsk & EVT_UART_NEW_CMD){
        OnCmd(&Uart);
        Uart.SignalCmdProcessed();
    }

 // ==== USB connected/disconnected ====
    if(WasExternal and !ExternalPwrOn()) {
        WasExternal = false;
        Usb.Shutdown();
        MassStorage.Reset();
        chSysLock();
        Clk.SetFreq12Mhz();
        Clk.InitSysTick();
        chSysUnlock();
        SndList.PlayRandomFileFromDir("0:\\");
        Uart.Printf("\rUsb Off");
    }
    else if(!WasExternal and ExternalPwrOn()) {
        WasExternal = true;
        Sound.Stop();
        chSysLock();
        Clk.SetFreq48Mhz();
        Clk.InitSysTick();
        chSysUnlock();
        Usb.Init();
        chThdSleepMilliseconds(540);
        Usb.Connect();
        Uart.Printf("\rUsb On");
    }

    } // while true
}

void App_t::OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t dw32 = 0;  // May be unused in some configurations
    Uart.Printf("\r New Cmd: %S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(OK);
    }

    else PShell->Ack(CMD_UNKNOWN);
}

