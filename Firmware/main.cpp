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
#include "SimpleSensors.h"
#include "buttons.h"
//#include "Sequences.h"
//#include "WS2812BforSPI.h"
//#include "WS2812BforTMR.h"
//#include "adc_f2.h"

App_t App;
SndList_t SndList;
Periphy_t Periphy;
PinInput_t ExternalPWR(ExternalPWR_Pin);
PinInput_t Box1Opened(Sensor1_Pin);
PinInput_t Box2Opened(Sensor2_Pin);
PinOutput_t BattMeasureSW(BattMeasSW_Pin);


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
    App.InitThread();

    // ==== Init Hard & Soft ====
    Uart.Init(115200, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN);
    Uart.Printf("\r%S AHB freq=%uMHz\r", BOARD_NAME, Clk.AHBFreqHz/1000000);
    // Report problem with clock if any
    if(ClkResult) Uart.Printf("Clock failure\r");

    // USB related
    MassStorage.Init();

    // Battery: ADC
    Adc.Init();
    BattMeasureSW.Init();
    BattMeasureSW.SetHi();
    PinSetupAnalog(BattMeas_Pin);

    // Setup inputs
    PinSensors.Init();
    // Setup outputs
    Periphy.InitSwich();

    // ==== Main cycle ====
    App.PowerON();
    App.ITask();
}


void App_t::PowerON() {
    Periphy.ON();
    chThdSleepMilliseconds(300);    // Let power to stabilize
    SD.Init();      // No Delay power
    Sound.AmpfOn();
    Sound.Init();
    Sound.RegisterAppThd(chThdSelf());

    if(Sleep::WasInStandby()) {
        Uart.Printf("\rWasStandby"); // WakeUp
        Sleep::DisableWakeup1Pin();
        Sleep::ClearStandbyFlag();
        SndList.SetPreviousTrack(BackupSpc::ReadBackupRegister(TrackNumberBKP));
        Sound.SetVolume(BackupSpc::ReadBackupRegister(VolumeBKP));
//        Uart.Printf("\r Load TrackNumber: %u", BackupSpc::ReadBackupRegister(TrackNumberBKP));
        BackupSpc::DisableBackupAccess();
    }
    else {
        Uart.Printf("\rPowerON");
        Sound.SetVolume(200);
    }

    SndList.PlayRandomFileFromDir("0:\\");
}

__attribute__ ((__noreturn__))
void App_t::ITask() {

#if 0 // Создать файл (проба)
    FIL File;
    FRESULT Rslt = f_open(&File, "ID_Store.ini",FA_READ+FA_OPEN_EXISTING+FA_CREATE_NEW);//
    if(Rslt != FR_OK) {
        if (Rslt == FR_NO_FILE) Uart.Printf("\r%S: not found", "ID_Store.ini");
        else Uart.Printf("\r%S: openFile error: %u", "ID_Store.ini", Rslt);
     }
    chThdSleepMilliseconds(999);
#endif


while(true) {
//    eventmask_t EvtMsk = chEvtWaitAnyTimeout(EVT_PLAY_ENDS, MS2ST(306));
    eventmask_t EvtMsk = chEvtWaitAny(ALL_EVENTS);

    if(EvtMsk & EVT_PLAY_ENDS) {
        if (!ExternalPWR.IsHi()){
//        if (!Periphy._5V_is_here()){
            SndList.PlayRandomFileFromDir("0:\\");
        }
    }

    if(EvtMsk & EVT_BUTTONS) {
        BtnEvtInfo_t EInfo;
        while(BtnGetEvt(&EInfo) == OK) {
            if(EInfo.Type == bePress) {
                switch(EInfo.BtnID[0]) {
                    case VolUpIndex:
                        Sound.VolumeIncrease();
                        break;
                    case VolDownIndex:
                        Sound.VolumeDecrease();
                        break;
                }
            }
            else if(EInfo.Type == beLongPress) {
                switch(EInfo.BtnID[0]) {
                    case VolUpIndex:
                        Sound.VolumeIncrease();
                        break;
                    case VolDownIndex:
                        Sound.VolumeDecrease();
                        break;
                }
            }
            else if(EInfo.Type == beRelease) {
//                Uart.Printf("\r Release");
//                Uart.Printf("\r BtnID %u", EInfo.BtnID[0]);
            }
        }
    }

    if(EvtMsk & EVT_BOX1_CLOSED) {
        if (!Box2Opened.IsHi() and !ExternalPWR.IsHi()){
            ShutDown();
        }
    }
    if(EvtMsk & EVT_BOX2_CLOSED) {
        if (!Box1Opened.IsHi() and !ExternalPWR.IsHi()){
            ShutDown();
        }
    }

    if(EvtMsk & EVT_ADC_DONE) {
        uint16_t BatAdc = 2 * Adc.GetResult(BAT_CHNL); // to count R divider
        uint8_t NewBatPercent = mV2PercentLiIon(BatAdc);
        Uart.Printf("mV=%u; percent=%u\r", BatAdc, NewBatPercent);
        Adc.DisableVref();
    }

 // ==== USB connected/disconnected ====
    if(EvtMsk & EVT_USB_CONNECTED) {
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
    if(EvtMsk & EVT_USB_DISCONNECTED) {
        Usb.Shutdown();
        MassStorage.Reset();
        chSysLock();
        Clk.SetFreq12Mhz();
        Clk.InitSysTick();
        chSysUnlock();
        Uart.Printf("\rUsb Off");
        if (!Box1Opened.IsHi() and !Box2Opened.IsHi())
            ShutDown();
        else{
            //        SD.Init();
            SndList.PlayRandomFileFromDir("0:\\");
        }
    }

    if(EvtMsk & EVT_UART_NEW_CMD) {
        OnCmd(&Uart);
        Uart.SignalCmdProcessed();
    }

    } // while true
}

void App_t::OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t Data = 0;  // May be unused in some configurations
    Uart.Printf("\r New Cmd: %S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(OK);
    }
    else if(PCmd->NameIs("Next")) {
        SndList.PlayRandomFileFromDir("0:\\");
    }
    else if(PCmd->NameIs("PerON")) {
        PowerON();
    }
    else if(PCmd->NameIs("PerOFF")) {
        Sound.Shutdown();
        Periphy.OFF();
    }
    else if(PCmd->NameIs("RUN_ADC")) {
        Adc.EnableVref();
        BattMeasureSW.SetLo(); // Connect R divider to GND
        chThdSleepMicroseconds(100);
        Adc.StartMeasurement();
//        chEvtWaitAny(EVT_ADC_DONE);
//        Adc.StartMeasurement();

//        Adc.Measure();
//        Uart.Printf("\r ADC: = %u", Adc.Result[0]);
    }
    else if(PCmd->NameIs("VolS")) {
        if(PCmd->GetNextInt32(&Data) == OK) {
            Uart.Printf("\r Data=%d", Data);
            Sound.SetVolume(Data);
        }
    }
    else if(PCmd->NameIs("VolG")) {
        Uart.Printf(" GetVolume %u", Sound.GetVolume());
    }

    else PShell->Ack(CMD_UNKNOWN);
}

//* TmrKL_t TmrCheckBtn {MS2ST(54), EVT_BUTTONS, tktPeriodic};
//* TmrCheckBtn.InitAndStart(chThdGetSelfX());
void App_t::ShutDown() {
//    Sound.Shutdown();
    Sound.Stop();
    chThdSleepMilliseconds(700);
    if (!Box1Opened.IsHi() and !Box2Opened.IsHi() and !ExternalPWR.IsHi()){
        chSysLock();
        Uart.PrintfNow("\rShutDown");
        Sound.Shutdown();
        Periphy.OFF();
        BackupSpc::EnableBackupAccess();
        BackupSpc::WriteBackupRegister(TrackNumberBKP, SndList.GetTrackNumber());
        BackupSpc::WriteBackupRegister(VolumeBKP, Sound.GetVolume());
        Sleep::EnableWakeup1Pin();
        Sleep::EnterStandby();
        chSysUnlock();
    }
//    else
//        SndList.PlayRandomFileFromDir("0:\\");
}


//void App_t::EnterState(AppState_t NewState) {
//    switch(NewState) {
//        case asDefault:
//
//            break;
//        default: break;
//    }
//    SystemState = NewState;
//}


// Snsors
void Process5VSns(PinSnsState_t *PState, uint32_t Len) {
    if(PState[0] == pssRising) App.SignalEvt(EVT_USB_CONNECTED);
    else if(PState[0] == pssFalling) App.SignalEvt(EVT_USB_DISCONNECTED);
}
void Process3VSns1(PinSnsState_t *PState, uint32_t Len) {
    if(PState[0] == pssFalling) App.SignalEvt(EVT_BOX1_CLOSED);
}
void Process3VSns2(PinSnsState_t *PState, uint32_t Len) {
    if(PState[0] == pssFalling) App.SignalEvt(EVT_BOX2_CLOSED);
}
